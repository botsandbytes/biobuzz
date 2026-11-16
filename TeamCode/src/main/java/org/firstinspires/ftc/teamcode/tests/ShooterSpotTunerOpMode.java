package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.Nullable;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.auto.PathUtil;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Finds the fixed shooting spot and its flywheel speed by hand.
 *
 * <p>Starts in the TeleOp reset corner ({@code teleop.poses.red.reset}: back on the audience wall,
 * left side on the RED wall) with the drive relaxed (zero power, motors floating), so the robot can
 * be pushed by hand while the Pinpoint keeps tracking it. Gamepad 1:
 *
 * <ul>
 *   <li>A: hold this position, spin up to {@link #shotRpm} (Panels, live), and run the transfer and
 *       intake when the flywheel is at speed.
 *   <li>B: stop shooting and relax the drive again.
 *   <li>D-pad: drive around the HIVE to a configured shooting spot ({@code teleop.poses.*.shoot_a /
 *       shoot_b}): down RED CELL A, up RED CELL B, left BLUE CELL A, right BLUE CELL B. Then A
 *       shoots from there.
 *   <li>BACK (relaxed): zero odometry in the reset corner.
 * </ul>
 *
 * <p>Telemetry's REPORT line gives the pose, the nearest CELL and its distance, and the speed: the
 * numbers to save as the shooting spot ({@code shoot_a}/{@code shoot_b}) and the shot speed.
 */
@Configurable
@TeleOp(name = "Shooter Spot Tuner", group = "Calibration")
// Hardware is set up inside runOpMode's try block, so not every path assigns every field.
@SuppressWarnings("NullAway.Init")
public final class ShooterSpotTunerOpMode extends LinearOpMode {
  /** Flywheel target while shooting, encoder ticks/s (DcMotorEx velocity units). Panels, live. */
  public static volatile double shotRpm = config.teleop.manual_rev_power * config.shooter.max_rpm;

  private enum Mode {
    RELAXED,
    DRIVING,
    HOLDING,
    SHOOTING
  }

  private record Spot(String button, String name, Pose pose) {}

  private Robot robot;
  private List<DcMotorEx> driveMotors;
  private List<Spot> spots;
  private Mode mode = Mode.RELAXED;
  private String status = "relaxed: push the robot where you want to shoot from";
  private @Nullable CommandBuilder driving;
  private Spot target;
  private List<Pose> route = List.of();
  private Pose routeStart = new Pose(0, 0, 0);
  private double appliedRpm = Double.NaN;
  private FieldModel blueField;

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT
    MatchProfile red = config.loadMatchProfile(Alliance.RED);
    MatchProfile blue = config.loadMatchProfile(Alliance.BLUE);
    blueField = new FieldModel(Alliance.BLUE);
    spots =
        List.of(
            new Spot("D-pad down", "RED CELL A spot", red.shootAPose()),
            new Spot("D-pad up", "RED CELL B spot", red.shootBPose()),
            new Spot("D-pad left", "BLUE CELL A spot", blue.shootAPose()),
            new Spot("D-pad right", "BLUE CELL B spot", blue.shootBPose()));
    target = spots.get(0);
    FieldManager panels = OpModeUtil.initPanelsField();
    try {
      robot = new Robot(hardwareMap, telemetry, red);
      robot.turret.setAimMode(Turret.AimMode.IDLE);
      robot.follower.setPose(red.resetPose());
      var d = config.pedro.drivetrain;
      driveMotors =
          List.of(
              hardwareMap.get(DcMotorEx.class, d.front_left_motor),
              hardwareMap.get(DcMotorEx.class, d.front_right_motor),
              hardwareMap.get(DcMotorEx.class, d.back_left_motor),
              hardwareMap.get(DcMotorEx.class, d.back_right_motor));

      while (opModeInInit()) {
        telemetry.addLine("Put the robot in the reset corner: back on the audience wall,");
        telemetry.addLine("left side on the RED wall, facing the rear wall. Then START.");
        telemetry.addLine("After START the drive is relaxed: push it to a spot, A to shoot.");
        telemetry.update();
        idle();
      }
      relax();

      boolean aPrev = false, bPrev = false, backPrev = false;
      boolean upPrev = false, downPrev = false, leftPrev = false, rightPrev = false;
      while (opModeIsActive()) {
        if (gamepad1.a && !aPrev) lockAndShoot();
        if (gamepad1.b && !bPrev) stopAndRelax();
        if (gamepad1.dpad_down && !downPrev) driveTo(spots.get(0));
        if (gamepad1.dpad_up && !upPrev) driveTo(spots.get(1));
        if (gamepad1.dpad_left && !leftPrev) driveTo(spots.get(2));
        if (gamepad1.dpad_right && !rightPrev) driveTo(spots.get(3));
        if (gamepad1.back && !backPrev && mode == Mode.RELAXED) {
          robot.follower.setPose(red.resetPose());
          status = "odometry zeroed in the reset corner";
        }
        aPrev = gamepad1.a;
        bPrev = gamepad1.b;
        backPrev = gamepad1.back;
        upPrev = gamepad1.dpad_up;
        downPrev = gamepad1.dpad_down;
        leftPrev = gamepad1.dpad_left;
        rightPrev = gamepad1.dpad_right;

        // Panels edits reach the flywheel at once while shooting.
        if (mode == Mode.SHOOTING) {
          applyShotSpeed();
        }
        checkArrival();

        robot.update();
        show();
        if (panels != null) {
          DrawingUtil.drawFieldZones(panels, robot.sentinel);
          DrawingUtil.drawRoute(panels, routeStart, route);
          Nearest n = nearest();
          OpModeUtil.drawRobot(panels, robot.follower, robot.turret, n.field().target(n.cell(), 0));
        }
      }
    } finally {
      if (robot != null) {
        CommandBuilder current = driving;
        if (current != null) current.cancel();
        robot.shotController.stopShot();
        robot.follower.stop();
        robot.shutdown();
      }
    }
  }

  private void applyShotSpeed() {
    double requested = shotRpm;
    double rpm =
        Double.isFinite(requested) ? Math.clamp(requested, 0.0, config.shooter.max_rpm) : 0.0;
    if (rpm == appliedRpm) {
      return;
    }
    appliedRpm = rpm;
    if (rpm > 0.0) {
      // Start a new feed gate at this target so a speed change cannot be counted as a ball.
      robot.shotController.startShot(rpm / config.shooter.max_rpm, false, false, false);
    } else {
      robot.shotController.stopShot();
    }
  }

  /** Drive at zero power with the motors floating, so the robot can be pushed by hand. */
  private void relax() {
    robot.follower.manual(0, 0, 0);
    for (DcMotorEx m : driveMotors) {
      m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    mode = Mode.RELAXED;
  }

  private void brake() {
    for (DcMotorEx m : driveMotors) {
      m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
  }

  private void cancelDriving() {
    CommandBuilder current = driving;
    if (current != null) {
      current.cancel();
      driving = null;
    }
  }

  /** Hold here, spin up, and feed with both actuators when the flywheel is at speed. */
  private void lockAndShoot() {
    Pose here = robot.follower.pose();
    if (!ShotInputs.isFinitePose(here)) {
      stopAndRelax();
      status = "pose unavailable: reset odometry in the corner";
      return;
    }
    cancelDriving();
    brake();
    robot.follower.hold(here);
    robot.intake.resumeTransfer();
    appliedRpm = Double.NaN;
    applyShotSpeed();
    mode = Mode.SHOOTING;
    status = "SHOOTING from here: tune shotRpm in Panels, B to stop";
  }

  private void stopAndRelax() {
    cancelDriving();
    robot.shotController.stopShot();
    relax();
    status = "relaxed: push the robot, A to shoot";
  }

  private void driveTo(Spot spot) {
    cancelDriving();
    robot.shotController.stopShot();
    Pose here = robot.follower.pose();
    if (!ShotInputs.isFinitePose(here)) {
      relax();
      status = "pose unavailable: reset odometry in the corner";
      return;
    }
    brake();
    // The shortest route around the HIVE frame; it may cross the centre line for BLUE's spots.
    List<Pose> planned = robot.sentinel.routeAround(here, spot.pose(), false);
    target = spot;
    routeStart = here;
    route = planned;
    if (planned.isEmpty()) {
      robot.follower.hold(here);
      mode = Mode.HOLDING;
      status = "NO CLEAR ROUTE to the " + spot.name() + ": push it there instead";
      return;
    }
    CommandBuilder legs =
        PathUtil.followAround(robot.follower, robot.sentinel, here, planned, false);
    legs.schedule();
    driving = legs;
    mode = Mode.DRIVING;
    status = "driving to the " + spot.name() + " (B stops)";
  }

  private void checkArrival() {
    CommandBuilder current = driving;
    if (mode != Mode.DRIVING || current == null || Scheduler.isScheduled(current)) {
      return;
    }
    driving = null;
    robot.follower.hold(target.pose());
    mode = Mode.HOLDING;
    status = "at the " + target.name() + ": A to shoot from here";
  }

  private record Nearest(String name, FieldModel field, HiveCell cell, double distance) {}

  /**
   * The CELL opening nearest the turret axis (any alliance, as if upward-facing) and its distance.
   */
  private Nearest nearest() {
    Pose pivot = Turret.pivotPose(robot.follower.pose());
    FieldModel redField = robot.field;
    Nearest best = new Nearest("?", redField, HiveCell.A, Double.POSITIVE_INFINITY);
    for (FieldModel f : List.of(redField, blueField)) {
      for (HiveCell cell : HiveCell.values()) {
        FieldModel.Opening o = f.upOpening(cell);
        double d = Math.hypot(o.x() - pivot.x(), o.y() - pivot.y());
        if (d < best.distance()) {
          best = new Nearest(f.alliance() + " CELL " + cell, f, cell, d);
        }
      }
    }
    return best;
  }

  private void show() {
    Pose p = robot.follower.pose();
    Nearest n = nearest();
    double targetRpm = mode == Mode.SHOOTING ? appliedRpm : 0.0;
    telemetry.addLine(
        String.format(
            Locale.ROOT,
            "REPORT  x=%.1f  y=%.1f  heading=%.0f  %s %.1f in  rpm=%.0f",
            p.x(),
            p.y(),
            Math.toDegrees(p.heading()),
            n.name(),
            n.distance(),
            mode == Mode.SHOOTING ? targetRpm : shotRpm));
    telemetry.addData("Mode", mode);
    telemetry.addData("Status", status);
    telemetry.addData(
        "Flywheel",
        "%.0f / %.0f ticks/s (manual_rev_power %.3f)",
        robot.shotController.getFlywheelVelocity(),
        targetRpm,
        targetRpm / config.shooter.max_rpm);
    telemetry.addData("Shot gate", robot.shotController.getGateReason());
    telemetry.addLine("A: hold + shoot | B: stop + relax | BACK (relaxed): zero in the corner");
    for (Spot s : spots) {
      telemetry.addLine(
          String.format(
              Locale.ROOT,
              "%s: %s (%.1f, %.1f, %.0f)",
              s.button(),
              s.name(),
              s.pose().x(),
              s.pose().y(),
              Math.toDegrees(s.pose().heading())));
    }
    telemetry.update();
  }
}
