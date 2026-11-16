package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.Nullable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Locale;
import java.util.function.BiFunction;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ballistics.ShotSolver;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.ballistics.ShotTimeTable;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.FlywheelDipDetector;

/**
 * Owns the shot lifecycle: aims at the upward-facing CELL, spins up, and feeds only while every
 * gate agrees.
 *
 * <p>The target is never a fixed field point: it is whichever of the alliance's two CELLS the
 * {@link HiveTracker} believes is upward-facing, and it changes with every TIP. Each published
 * {@link ShotInputs} carries the tracker's epoch and each {@link ShotSolution} echoes it back; a
 * solution from an older epoch (or for the other SCORING ELEMENT) never moves the hood, never sets
 * the flywheel and never authorizes a feed. Balls already in the air cannot be recalled, so the
 * feed also pauses until the tracker reports the HIVE settled after a TIP.
 *
 * <p>The solve itself runs on a background thread ({@code solverThread}); {@link #periodic()} only
 * publishes inputs and reads the latest completed solution.
 */
public class ShotController {
  private final Shooter shooter;
  private final Turret turret;
  private final Intake intake;
  private final Supplier<Pose> poseSupplier;
  private final Supplier<Velocity> velocitySupplier;
  private final @Nullable Casablanca casablanca;
  private final FieldModel field;
  private final HiveTracker hive;
  private final @Nullable Telemetry telemetry;

  private boolean active = false;
  private boolean checkAlignment = false;
  private boolean useSolvedRpm = false;
  private boolean requireValidSolution = true;
  private BallType ballType = BallType.POLLEN;

  /** Latching state of the feed gate; see {@link #updateFeedGate()}. */
  private boolean feeding = false;

  // TeleOp's operator owns the intake (see setOperatorIntake). Main-thread only.
  private boolean operatorIntake = false;

  // Where a manual shot drove to (manualShotCommand); it feeds only once the robot is there. Null
  // for every other shot, which holds where it already is. Main-thread only.
  private @Nullable Pose shotPosition = null;

  /** Whether the last {@link #periodic()} actually fed the shooter, i.e. every gate agreed. */
  private boolean feedCommanded = false;

  /** First gate that blocked the last feed decision, for driver telemetry. */
  private String gateReason = "idle";

  // Live ball counting, from the flywheel dip each one costs. Built fresh per shot so a hot-pushed
  // config.yaml threshold takes effect on the very next shot.
  private FlywheelDipDetector ballDetector = newBallDetector();
  private boolean ballDetectionArmed = false;
  private int ballsFired = 0;

  /** Flywheel setpoint most recently commanded from a solution; NaN until the first one lands. */
  private double commandedSolvedRpm = Double.NaN;

  private final ElapsedTime timer = new ElapsedTime();

  // Solver thread fields
  private final Object solveLock = new Object();
  private @Nullable ShotInputs pendingInputs;
  private long solveGeneration = 0;
  private final BiFunction<ShotInputs, Double, ShotSolution> solve;
  private double lastFlightTime = 0.0;
  private volatile ShotSolution lastSolution = ShotSolution.invalid("Not initialized");
  private volatile boolean solving = false;
  private volatile long lastSolveDurationMs = 0;
  private volatile boolean running = true;
  private final Thread solverThread;

  public ShotController(
      Shooter shooter,
      Turret turret,
      Intake intake,
      Supplier<Pose> poseSupplier,
      Supplier<Velocity> velocitySupplier,
      @Nullable Casablanca casablanca,
      FieldModel field,
      HiveTracker hive,
      @Nullable Telemetry telemetry) {
    this(
        shooter,
        turret,
        intake,
        poseSupplier,
        velocitySupplier,
        casablanca,
        field,
        hive,
        telemetry,
        (inputs, previousFlightTime) ->
            ShotSolver.solve(
                inputs,
                ShotTable.fromConfig(inputs.ballType()),
                BallisticsParameters.fromConfig(),
                previousFlightTime));
  }

  ShotController(
      Shooter shooter,
      Turret turret,
      Intake intake,
      Supplier<Pose> poseSupplier,
      Supplier<Velocity> velocitySupplier,
      @Nullable Casablanca casablanca,
      FieldModel field,
      HiveTracker hive,
      @Nullable Telemetry telemetry,
      BiFunction<ShotInputs, Double, ShotSolution> solve) {
    this.solve = solve;
    this.shooter = shooter;
    this.turret = turret;
    this.intake = intake;
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;
    this.casablanca = casablanca;
    this.field = field;
    this.hive = hive;
    this.telemetry = telemetry;

    solverThread = new Thread(this::solveLoop, "ShotSolver");
    solverThread.setDaemon(true);
    solverThread.start();
  }

  public void startShot(double power, boolean checkAlignment) {
    startShot(power, checkAlignment, checkAlignment);
  }

  public void startShot(double power, boolean checkAlignment, boolean useSolvedRpm) {
    startShot(power, checkAlignment, useSolvedRpm, true);
  }

  public void startShot(
      double power, boolean checkAlignment, boolean useSolvedRpm, boolean requireValidSolution) {
    this.active = true;
    this.checkAlignment = checkAlignment;
    this.useSolvedRpm = useSolvedRpm;
    this.requireValidSolution = requireValidSolution;
    this.commandedSolvedRpm = Double.NaN;
    this.feeding = false;
    this.feedCommanded = false;
    this.shotPosition = null;
    this.timer.reset();
    this.ballDetector = newBallDetector();
    this.ballDetectionArmed = false;
    this.ballsFired = 0;
    // A solution left over from before this shot was solved for whatever pose and HIVE state held
    // then. Nothing may fire until the solver has answered for now.
    invalidateSolve();
    intake.setShotTransferGated(true);
    shooter.setTargetPower(power);
    if (checkAlignment && turret != null && turret.isEnabled()) {
      turret.setAimMode(Turret.AimMode.AIM_AT_TARGET);
    }
  }

  /**
   * TeleOp only: the operator is running the intake by hand (the A toggle, or a held eject). While
   * set, a closed feed gate leaves the intake to the operator instead of stopping it, so the
   * operator can feed a shot the gate cannot clear yet (no calibrated shot table, no trusted pose),
   * at their own call, exactly as operator X plus A always could. An open gate still feeds by
   * itself, and a lost pose still stops the intake. Autonomous never sets it.
   */
  public void setOperatorIntake(boolean operatorOwnsIntake) {
    operatorIntake = operatorOwnsIntake;
  }

  public void stopShot() {
    this.active = false;
    this.feedCommanded = false;
    this.shotPosition = null;
    invalidateSolve();
    shooter.setTargetPower(0.0);
    intake.stop();
    intake.setShotTransferGated(false);
    turret.setTargetAzimuth(Double.NaN);
    if (casablanca != null) {
      casablanca.setArmedAimTarget(0.0, false);
    }
    if (checkAlignment && turret.isEnabled()) {
      turret.setHoldAngle(0.0);
      turret.setAimMode(Turret.AimMode.HOLD);
    }
    this.checkAlignment = false;
    this.useSolvedRpm = false;
    this.commandedSolvedRpm = Double.NaN;
    this.feeding = false;
    this.gateReason = "idle";
  }

  public void shutdown() {
    running = false;
    synchronized (solveLock) {
      solveLock.notifyAll();
    }
    solverThread.interrupt();
    try {
      solverThread.join(200);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  private void invalidateSolve() {
    synchronized (solveLock) {
      solveGeneration++;
      pendingInputs = null;
      lastSolution = ShotSolution.invalid("Solving");
    }
  }

  private void publishInputs(ShotInputs inputs) {
    synchronized (solveLock) {
      pendingInputs = inputs;
      solveLock.notify();
    }
  }

  private void solveLoop() {
    while (running) {
      ShotInputs inputs;
      long generation;
      synchronized (solveLock) {
        while (running && pendingInputs == null) {
          try {
            solveLock.wait();
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return;
          }
        }
        if (!running) {
          return;
        }
        generation = solveGeneration;
        inputs = pendingInputs;
        pendingInputs = null;
      }

      solving = true;
      long startNanos = System.nanoTime();

      ShotSolution result;
      try {
        result = solve.apply(inputs, lastFlightTime);
      } catch (RuntimeException e) {
        result = ShotSolution.invalid("Shot table error: " + e.getMessage());
      }
      if (result.predictedFlightTimeSec() > 0) {
        lastFlightTime = result.predictedFlightTimeSec();
      }

      lastSolveDurationMs = (System.nanoTime() - startNanos) / 1_000_000;
      synchronized (solveLock) {
        if (running && generation == solveGeneration) {
          lastSolution = result;
        }
      }
      solving = false;
    }
  }

  /**
   * Hold position and shoot at the upward CELL until cancelled, feeding only when every gate
   * passes. Autonomous wraps it in {@link #volleyCommand(Follower, int)}.
   */
  public CommandBuilder aimAndShootCommand(Follower follower) {
    return Command.build()
        .setStart(
            () -> {
              follower.hold(shotHoldPose(follower.pose()));
              startShot(Shooter.constantPower(), true);
            })
        .setEnd(interrupted -> stopShot())
        .requiring(follower, shooter, turret, intake);
  }

  /**
   * TeleOp's aim-and-shoot, until cancelled: drives forward or back to {@code
   * teleop.shot_distance_in} from the upward CELL if it is not there ({@link #rangedShotHoldPose}),
   * turns to face it, spins the flywheel to the shot table's speed for that spot ({@link
   * #tablePower}), and runs the intake once the flywheel is in band, the robot is aimed and it has
   * reached that position. A manual shot (the drivers' call, like any fixed-power shot): it skips
   * the solution and HIVE-settle gates, so it fires before the shot table is marked calibrated. It
   * still only aims at the CELL the HIVE tracker believes is up (G417).
   */
  public CommandBuilder manualShotCommand(Follower follower) {
    return Command.build()
        .setStart(
            () -> {
              Pose position = rangedShotHoldPose(follower.pose());
              follower.hold(position);
              // Pinned for the whole shot (useSolvedRpm false): the solver's live answer would
              // chase the distance while the robot drives there.
              startShot(tablePower(position), true, false, false);
              shotPosition = position;
            })
        .setEnd(interrupted -> stopShot())
        .requiring(follower, shooter, turret, intake);
  }

  /**
   * TeleOp's shot from the one fixed spot, until cancelled: spins the flywheel to {@code power} and
   * runs the intake as soon as the flywheel is in band. No driving, no aiming, no shot table: the
   * drivers put the robot on the spot (driver LT/RT drive there and hold it). It leaves the drive
   * and the turret alone, so that hold keeps running while it shoots. Still the CELL the HIVE
   * tracker believes is up is the only one the spot faces (G417).
   */
  public CommandBuilder fixedShotCommand(double power) {
    return Command.build()
        .setStart(() -> startShot(power, false, false, false))
        .setEnd(interrupted -> stopShot())
        .requiring(shooter, intake);
  }

  /**
   * Flywheel power for a shot from {@code position}: the shot table's speed for the current ball
   * type at that distance from the upward CELL, interpolated, clamped to the table's ends and to
   * what the shooter can command. The table's calibrated flag only gates automatic feeding, so the
   * rows are used either way. {@link Shooter#constantPower()} if the table cannot be read.
   */
  public double tablePower(Pose position) {
    try {
      double rpm = ShotTable.fromConfig(ballType).lookup(distanceToTarget(position)).rpm();
      double power = rpm / config.shooter.max_rpm;
      return Double.isFinite(power) ? Math.clamp(power, 0.0, 1.0) : Shooter.constantPower();
    } catch (RuntimeException e) {
      return Shooter.constantPower();
    }
  }

  /**
   * Where TeleOp's manual shot holds: on the line from the upward CELL's opening through the turret
   * axis, with the axis {@code teleop.shot_distance_in} from the opening, facing it, so a
   * fixed-power shot is always taken from the distance it was tuned at. The robot only drives
   * (straight toward or away from the CELL) when the axis is more than {@code
   * teleop.shot_position_tolerance_in} off that distance; otherwise this is {@link #shotHoldPose}.
   * While footprint safety is on, a drive that would touch the HIVE frame or a wall is refused and
   * it also falls back to {@link #shotHoldPose}.
   */
  public Pose rangedShotHoldPose(Pose here) {
    CellTarget target = currentTarget();
    Pose pivot = Turret.pivotPose(here);
    double dx = pivot.x() - target.x();
    double dy = pivot.y() - target.y();
    double distance = Math.hypot(dx, dy);
    var t = config.teleop;
    if (!(distance > 0.0)
        || Math.abs(distance - t.shot_distance_in) <= t.shot_position_tolerance_in) {
      return shotHoldPose(here);
    }
    double scale = t.shot_distance_in / distance;
    double facing = Math.atan2(-dy, -dx);
    Pose goal =
        Turret.robotPoseForPivot(
            new Pose(target.x() + dx * scale, target.y() + dy * scale, facing));
    if (casablanca != null
        && casablanca.sentinel().footprintSafetyEnabled()
        && casablanca.sentinel().routeProblem(straightDrive(here, goal), false).isPresent()) {
      return shotHoldPose(here);
    }
    return goal;
  }

  /** Poses every inch along a straight drive, turning evenly the short way, for route checks. */
  static java.util.List<Pose> straightDrive(Pose from, Pose to) {
    int steps = Math.max(2, (int) Math.ceil(from.distance(to)));
    double turn = AngleUnit.normalizeRadians(to.heading() - from.heading());
    java.util.List<Pose> samples = new java.util.ArrayList<>(steps + 1);
    for (int i = 0; i <= steps; i++) {
      double f = (double) i / steps;
      samples.add(
          new Pose(
              from.x() + f * (to.x() - from.x()),
              from.y() + f * (to.y() - from.y()),
              from.heading() + f * turn));
    }
    return samples;
  }

  /**
   * Where to hold the chassis for a shot. Normally right where it is, and the turret does the
   * aiming. If the turret is disabled, or the CELL is outside its travel, holding the current
   * heading could never become aimed, so the robot turns in place to face the CELL — unless
   * footprint safety is on and that rotation would swing the footprint into the HIVE frame, in
   * which case it holds and the shot gate explains.
   */
  public Pose shotHoldPose(Pose here) {
    CellTarget target = currentTarget();
    Pose pivot = Turret.pivotPose(here);
    double bearing = Math.atan2(target.y() - pivot.y(), target.x() - pivot.x());
    double turretAngle = Math.toDegrees(AngleUnit.normalizeRadians(bearing - here.heading()));
    var travel = config.turret.travel;
    boolean turretCovers =
        turret.isEnabled() && turretAngle >= travel.min_angle && turretAngle <= travel.max_angle;
    if (turretCovers) {
      return here;
    }
    if (casablanca != null
        && casablanca.sentinel().footprintSafetyEnabled()
        && !casablanca.sentinel().isRotationClear(here, bearing)) {
      return here;
    }
    return new Pose(here.x(), here.y(), bearing);
  }

  /**
   * One autonomous volley: shoot until {@code expectedBalls} shots are counted or the inventory is
   * known to be empty. With {@code auto.shot_window_enabled} it also ends when the measured shoot
   * window for this distance expires, even if the dip detector missed a ball; without it only the
   * count ends it, and the park guard still ends the routine at its park deadline.
   *
   * <p>The count ticks on the first sample of a ball's flywheel dip, while that ball is still on
   * the wheel. Ending the volley there braked the flywheel under the last ball, so the volley also
   * waits for the wheel to recover from that dip.
   */
  public CommandBuilder volleyCommand(Follower follower, int expectedBalls) {
    return lazy(() -> volleyOf(aimAndShootCommand(follower), follower, expectedBalls));
  }

  /**
   * One autonomous volley at a calibrated pose and speed. Waits to reach the pose before starting
   * the shooter and transfer, feeds when the flywheel is in band, and ends if the robot drifts or
   * the HIVE tracker changes away from the requested CELL (G417).
   */
  public CommandBuilder fixedVolleyCommand(
      Follower follower, Pose position, HiveCell cell, double power, int expectedBalls) {
    return race(
            waitUntil(() -> hive.upCell() != cell),
            sequential(
                instant(() -> follower.hold(position)),
                waitUntil(() -> atFixedShotPose(follower.pose(), position)),
                lazy(
                    () ->
                        race(
                            waitUntil(() -> !atFixedShotPose(follower.pose(), position)),
                            volleyOf(fixedShotCommand(power), follower, expectedBalls)))))
        .requiring(follower, shooter, intake);
  }

  private static boolean atFixedShotPose(Pose current, Pose target) {
    return ShotInputs.isFinitePose(current)
        && current.distance(target) <= config.teleop.shot_position_tolerance_in
        && Math.abs(AngleUnit.normalizeRadians(current.heading() - target.heading()))
            <= Math.toRadians(config.casablanca.heading_lock.error_deadband_deg);
  }

  /** {@code shot} until the count (or, with the shoot window on, the window) ends the volley. */
  private Command volleyOf(Command shot, Follower follower, int expectedBalls) {
    var counted =
        sequential(
            waitUntil(
                () ->
                    ballsFired >= expectedBalls
                        || (ballsFired > 0 && intake.inventory().isEmpty())),
            waitUntil(() -> !ballDetector.isInDip()));
    if (!config.auto.shot_window_enabled) {
      return race(shot, counted);
    }
    return race(
        shot, counted, waitMs(ShotTimeTable.windowMsFor(distanceToTarget(follower.pose()))));
  }

  /** Horizontal distance from the turret pivot at {@code pose} to the upward CELL opening. */
  public double distanceToTarget(Pose pose) {
    CellTarget target = currentTarget();
    Pose pivot = Turret.pivotPose(pose);
    return Math.hypot(target.x() - pivot.x(), target.y() - pivot.y());
  }

  /** The upward CELL as the tracker believes it right now. */
  public CellTarget currentTarget() {
    return field.target(hive.upCell(), hive.epoch());
  }

  public boolean isActive() {
    return active;
  }

  public boolean isFeedCommanded() {
    return feedCommanded;
  }

  public int getBallsFired() {
    return ballsFired;
  }

  public String getGateReason() {
    return gateReason;
  }

  public BallType getBallType() {
    return ballType;
  }

  /**
   * Which shot table to use. Changing it invalidates the current solution. A ball type that is not
   * {@link #isSelectable selectable} falls back to POLLEN.
   */
  public void setBallType(BallType requested) {
    BallType ballType = isSelectable(requested) ? requested : BallType.POLLEN;
    if (this.ballType != ballType) {
      this.ballType = ballType;
      invalidateSolve();
      this.commandedSolvedRpm = Double.NaN;
    }
  }

  /** NECTAR is selectable only while {@code shooter.shot_table.nectar_enabled} is true. */
  public static boolean isSelectable(BallType type) {
    return type == BallType.POLLEN
        || (type == BallType.NECTAR && config.shooter.shot_table.nectar_enabled);
  }

  private static FlywheelDipDetector newBallDetector() {
    var bd = config.shooter.ball_detection;
    return new FlywheelDipDetector(
        bd.dip_fraction, bd.rebound_fraction, bd.baseline_alpha, bd.refractory_ms);
  }

  public ShotSolution getLastSolution() {
    return lastSolution;
  }

  /**
   * Seeds a solution (e.g. pre-solved at init for the first volley pose) to pre-position the hood.
   */
  public void setInitialSolution(ShotSolution solution) {
    if (solution != null) {
      this.lastSolution = solution;
      shooter.setTargetHoodPosition(solution.targetHoodPosition());
    }
  }

  /** True if {@code solution} was solved for the HIVE state and SCORING ELEMENT in force now. */
  public static boolean isCurrent(ShotSolution solution, int epoch, BallType ballType) {
    return FeedLaw.isCurrent(
        solution.targetEpoch(), epoch, ordinalOf(solution.ballType()), ordinalOf(ballType));
  }

  private static int ordinalOf(BallType type) {
    return type == null ? -1 : type.ordinal();
  }

  public double getFlywheelVelocity() {
    return Math.abs(shooter.getShooterVelocity());
  }

  public double getFlywheelTarget() {
    double commandedPower = shooter.getTargetPower();
    return commandedPower > 0
        ? Math.abs(config.shooter.max_rpm * commandedPower)
        : Math.abs(config.shooter.constant_rpm);
  }

  public boolean isFlywheelReady() {
    var shooterConfig = config.shooter;
    double target = getFlywheelTarget();
    double current = getFlywheelVelocity();
    return FeedLaw.inBand(
        current,
        target,
        shooterConfig.min_transfer_threshold,
        shooterConfig.max_velocity_threshold);
  }

  public double getFlywheelVelocityRatio() {
    double target = getFlywheelTarget();
    return FeedLaw.ratio(getFlywheelVelocity(), target);
  }

  public String getFlywheelGateDetail() {
    var shooterConfig = config.shooter;
    double ratio = getFlywheelVelocityRatio();
    if (ratio < shooterConfig.min_transfer_threshold) {
      return String.format(
          Locale.ROOT, "BELOW min (%.2fx < %.2f)", ratio, shooterConfig.min_transfer_threshold);
    }
    if (ratio > shooterConfig.max_velocity_threshold) {
      return String.format(
          Locale.ROOT, "ABOVE max (%.2fx > %.2f)", ratio, shooterConfig.max_velocity_threshold);
    }
    return String.format(Locale.ROOT, "PASS (%.2fx)", ratio);
  }

  private boolean updateFeedGate() {
    var shooterConfig = config.shooter;
    feeding =
        shouldFeed(
            getFlywheelVelocityRatio(),
            feeding,
            shooterConfig.min_transfer_threshold,
            shooterConfig.max_velocity_threshold,
            shooterConfig.feed_release_threshold);
    return feeding;
  }

  public static boolean shouldFeed(
      double velocityRatio,
      boolean currentlyFeeding,
      double armLow,
      double armHigh,
      double releaseLow) {
    return FeedLaw.shouldFeed(velocityRatio, currentlyFeeding, armLow, armHigh, releaseLow);
  }

  private void applySolvedRpm(ShotSolution solution) {
    if (!useSolvedRpm || !solution.isValid() || solution.targetRpm() <= 0) {
      return;
    }
    double rpm =
        Math.clamp(
            solution.targetRpm(),
            Math.min(
                config.shooter.ballistics.preferred_shot_rpm,
                config.shooter.ballistics.max_shot_rpm),
            Math.max(
                config.shooter.ballistics.preferred_shot_rpm,
                config.shooter.ballistics.max_shot_rpm));

    if (!Double.isNaN(commandedSolvedRpm)
        && Math.abs(rpm - commandedSolvedRpm) < config.shooter.ballistics.rpm_update_deadband) {
      return;
    }
    commandedSolvedRpm = rpm;
    // Commanded deceleration can exceed the ball-dip threshold. Re-arm at the new speed,
    // preserving the volley count, rather than treating that transient as a launched ball.
    ballDetectionArmed = false;
    ballDetector.reset();
    feeding = false;
    shooter.setTargetPower(rpm / config.shooter.max_rpm);
  }

  /**
   * Decides this loop's hood, flywheel, turret and feed targets. Runs before the subsystems' own
   * periodic() writes, so every target set here reaches hardware in the same loop.
   */
  public void periodic() {
    Pose pose = poseSupplier != null ? poseSupplier.get() : null;
    Velocity vel = velocitySupplier != null ? velocitySupplier.get() : Velocity.zero();
    CellTarget target = currentTarget();
    turret.setTargetPoint(target.x(), target.y());

    if (!ShotInputs.isFinitePose(pose) || !ShotInputs.isFiniteVelocity(vel)) {
      // An earlier solution may still be current by epoch/type while localization has failed.
      // Close the gate this loop, before applying any of its actuator targets, and require a
      // new solve after recovery instead of waiting for the background solver to notice.
      feeding = false;
      feedCommanded = false;
      gateReason = active ? "localization unavailable" : "idle";
      if (active) {
        intake.stop();
        invalidateSolve();
      }
      turret.setTargetAzimuth(Double.NaN);
      if (casablanca != null) {
        casablanca.setArmedAimTarget(0.0, false);
      }
      return;
    }

    boolean needSolve =
        active
            || checkAlignment
            || (turret != null
                && turret.isEnabled()
                && turret.getAimMode() == Turret.AimMode.AIM_AT_TARGET);

    ShotSolution solution = lastSolution;
    boolean solutionCurrent = isCurrent(solution, target.epoch(), ballType);

    if (pose != null && needSolve) {
      // Solve from the turret axis, which is where the ball leaves from and what the turret
      // rotates about; the robot centre can be inches away at BIOBUZZ ranges.
      Pose muzzle = Turret.pivotPose(pose);
      Velocity muzzleVelocity = Turret.pivotVelocity(pose, vel);
      publishInputs(new ShotInputs(muzzle, muzzleVelocity, target, ballType));

      // A fixed shot (pinned power, no solution required) owns its hood, as a calibration shot
      // does: the solver's answer for wherever the robot is now must not overwrite it.
      boolean fixedShot = active && !useSolvedRpm && !requireValidSolution;
      if (solutionCurrent && !fixedShot) {
        shooter.setTargetHoodPosition(solution.targetHoodPosition());
        applySolvedRpm(solution);
      }
      boolean aimable = solutionCurrent && solution.isValid();
      // Aim with the solver's (lead-compensated) azimuth whenever it is current, even when the shot
      // is not allowed yet. Switching to the raw bearing whenever validity flickered (speed or
      // range
      // gate, uncalibrated table) jumped the turret by the whole lead angle.
      boolean aimAzimuth = solutionCurrent && Double.isFinite(solution.targetAzimuthRad());
      turret.setTargetAzimuth(aimAzimuth ? solution.targetAzimuthRad() : Double.NaN);

      if (casablanca != null) {
        casablanca.setArmedAimTarget(
            solution.targetAzimuthRad(), active && checkAlignment && aimable);
      }
    }

    if (!active) {
      feeding = false;
      feedCommanded = false;
      return;
    }

    var shooterConfig = config.shooter;
    double targetFlywheelVelocity = getFlywheelTarget();

    boolean isFlywheelReady = updateFeedGate();

    if (!ballDetectionArmed && isFlywheelReady()) {
      ballDetectionArmed = true;
      ballDetector.reset();
    }
    if (ballDetectionArmed
        && ballDetector.update(getFlywheelVelocity(), (long) timer.milliseconds())) {
      ballsFired++;
      intake.inventory().onShot();
    }

    boolean isTurretAligned = true;
    boolean inShotPosition = true;
    if (checkAlignment && pose != null) {
      Pose position = shotPosition;
      inShotPosition =
          position == null || pose.distance(position) <= config.teleop.shot_position_tolerance_in;
      isTurretAligned = inShotPosition && turret.isAimed(pose);
    }

    boolean isSolutionValid =
        FeedLaw.solutionOk(requireValidSolution, solutionCurrent, solution.isValid());
    // Manual fixed-power shots (requireValidSolution false) are the drivers' call; automatic ones
    // also wait for the HIVE to settle after a TIP.
    boolean isHiveSettled = FeedLaw.hiveOk(requireValidSolution, hive.isSettled());

    if (!isFlywheelReady) {
      gateReason = "flywheel " + getFlywheelGateDetail();
    } else if (!inShotPosition) {
      gateReason = "driving to shooting distance";
    } else if (!isTurretAligned) {
      gateReason =
          turret.isEnabled() && turret.isTargetOutsideTravel()
              ? "CELL outside turret travel: turn the robot toward it"
              : "aiming";
    } else if (!isSolutionValid) {
      gateReason = solutionCurrent ? solution.validityReason() : "solving for current HIVE state";
    } else if (!isHiveSettled) {
      gateReason = "HIVE settling after TIP";
    } else {
      gateReason = "FEEDING";
    }

    if (telemetry != null) {
      telemetry.addData("Shot Target", "CELL %s (epoch %d)", target.cell(), target.epoch());
      telemetry.addData("Shot Gate", gateReason);
      telemetry.addData("Flywheel Arm Window", getFlywheelGateDetail());
      telemetry.addData(
          "Feed Latch",
          feeding
              ? String.format(
                  Locale.ROOT,
                  "FEEDING (releases below %.2fx)",
                  shooterConfig.feed_release_threshold)
              : "closed");
      telemetry.addData("Balls Fired", ballsFired);
      telemetry.addData("Ball Type", ballType);
      telemetry.addData("Solution Dist (in)", solution.distanceInches());
      telemetry.addData("Target Hood Pos", solution.targetHoodPosition());
      telemetry.addData("Solved RPM", solution.targetRpm());
      telemetry.addData("Commanded RPM", targetFlywheelVelocity);
      telemetry.addData("Solver Thinking", solving);
      telemetry.addData("Last Solve (ms)", lastSolveDurationMs);
    }

    feedCommanded =
        FeedLaw.gate(
            isFlywheelReady,
            isTurretAligned,
            requireValidSolution,
            solutionCurrent,
            solution.isValid(),
            hive.isSettled());
    if (feedCommanded) {
      intake.feed(shooterConfig.feed_intake_power);
    } else if (!operatorIntake) {
      intake.stop();
    }
  }
}
