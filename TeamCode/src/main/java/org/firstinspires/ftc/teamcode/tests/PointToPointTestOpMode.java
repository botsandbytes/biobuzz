package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.Nullable;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.DownAuto;
import org.firstinspires.ftc.teamcode.auto.PathUtil;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Automated point-to-point driving through every TeleOp pose, for testing the follower and the HIVE
 * detour planner.
 *
 * <p>Starts from the TeleOp reset corner ({@code teleop.poses.red.reset}: back on the audience
 * wall, left side on the RED alliance wall; TeleOp's driver BACK zeroes odometry there). The stops
 * are every pose TeleOp drives to, for both alliances: the start, both CELL shooting poses (all
 * four CELLS), park and the FLOWER pose. Each trip uses {@code Sentinel.routeAround} and holds the
 * reached stop. Routes can use the HIVE opening and cross the centre line. Wall-side stops are
 * shifted inward to the planned wall clearance.
 *
 * <p>Gamepad 1: X tours every stop in order, dwelling {@link #tourDwellMs} at each, and ends back
 * near the reset corner; D-pad up/down picks one stop and A drives there; Y drives back near the
 * reset corner; B ends a trip or tour and holds where it is; BACK re-zeroes odometry at the reset
 * corner (robot pushed into it).
 */
@Configurable
@TeleOp(name = "Point To Point Drive Test", group = "Test")
// Hardware is set up inside runOpMode's try block, so not every path assigns every field.
@SuppressWarnings("NullAway.Init")
public final class PointToPointTestOpMode extends LinearOpMode {
  /** How long a tour waits at each stop once it has arrived. Panels-tunable. */
  public static long tourDwellMs = 1000;

  private record Stop(String name, Pose pose) {}

  private Robot robot;
  private List<Stop> stops;
  private Stop resetCorner;
  private int selected = 0;
  private String status = "idle";
  private List<Pose> route = List.of();
  private Pose routeStart = new Pose(0, 0, 0);
  private @Nullable CommandBuilder driving;
  private Stop target;

  /** Index of the stop the tour is on, or -1 when not touring. */
  private int tourIndex = -1;

  /** When the tour may leave the stop it is holding; 0 while still driving to it. */
  private long dwellUntilMs = 0;

  private final List<String> tourLog = new ArrayList<>();

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT
    MatchProfile red = config.loadMatchProfile(Alliance.RED);
    MatchProfile blue = config.loadMatchProfile(Alliance.BLUE);
    resetCorner = new Stop("reset corner", red.resetPose());
    // One loop of the field from the reset corner: RED's rear side, RED's audience side, across to
    // BLUE's audience side, BLUE's rear side, and home.
    // Typed pose classes, so a renamed or removed key fails to compile instead of loading null.
    Pose upStart = ConfigLoader.loadMerged(config.UpAuto.class, "auto_poses.up.red", "auto").start;
    Pose parkOnlyStart =
        ConfigLoader.loadMerged(config.ParkOnlyAuto.class, "auto_poses.park_only.red", "auto")
            .start;
    stops =
        List.of(
            new Stop("RED Up AUTO start", upStart),
            new Stop("RED Down AUTO start", DownAuto.flipOfUp(upStart)),
            new Stop("RED Park Only start", parkOnlyStart),
            new Stop("RED shoot_b", red.shootBPose()),
            new Stop("RED park", red.parkPose()),
            new Stop("RED flower", red.flowerPose()),
            new Stop("RED shoot_a", red.shootAPose()),
            new Stop("RED start", red.startPose()),
            new Stop("BLUE shoot_b", blue.shootBPose()),
            new Stop("BLUE park", blue.parkPose()),
            new Stop("BLUE flower", blue.flowerPose()),
            new Stop("BLUE shoot_a", blue.shootAPose()),
            new Stop("BLUE start", blue.startPose()),
            resetCorner);
    target = stops.get(0);
    FieldManager panels = OpModeUtil.initPanelsField();
    try {
      robot = new Robot(hardwareMap, telemetry, red);
      robot.turret.setAimMode(Turret.AimMode.IDLE);
      robot.follower.setPose(red.resetPose());

      boolean upPrev = true, downPrev = true, aPrev = true, bPrev = true, xPrev = true;
      boolean yPrev = true, backPrev = true;
      while (!isStopRequested()) {
        boolean running = isStarted();
        boolean idle = driving == null && tourIndex < 0;

        if (gamepad1.dpad_up && !upPrev) selected = (selected + stops.size() - 1) % stops.size();
        if (gamepad1.dpad_down && !downPrev) selected = (selected + 1) % stops.size();
        if (running && idle && gamepad1.a && !aPrev) driveTo(stops.get(selected));
        if (running && idle && gamepad1.y && !yPrev) driveTo(resetCorner);
        if (running && idle && gamepad1.x && !xPrev) startTour();
        if (running && gamepad1.b && !bPrev) stopHere();
        if (idle && gamepad1.back && !backPrev) {
          robot.follower.setPose(red.resetPose());
          status = "odometry zeroed at the reset corner";
        }
        upPrev = gamepad1.dpad_up;
        downPrev = gamepad1.dpad_down;
        aPrev = gamepad1.a;
        bPrev = gamepad1.b;
        xPrev = gamepad1.x;
        yPrev = gamepad1.y;
        backPrev = gamepad1.back;

        if (running) {
          robot.update(); // Nothing drives during INIT.
          checkArrival();
          advanceTour();
        }
        show(running);
        if (panels != null) {
          DrawingUtil.drawFieldZones(panels, robot.sentinel);
          DrawingUtil.drawRoute(panels, routeStart, route);
          OpModeUtil.drawRobot(
              panels, robot.follower, robot.turret, robot.shotController.currentTarget());
        }
        idle();
      }
    } finally {
      if (robot != null) {
        CommandBuilder current = driving;
        if (current != null) current.cancel();
        robot.follower.stop();
        robot.shutdown();
      }
    }
  }

  /** A finished route: hold the stop itself (the last leg ends on its path) and report. */
  private void checkArrival() {
    CommandBuilder current = driving;
    if (current == null || Scheduler.isScheduled(current)) {
      return;
    }
    driving = null;
    robot.follower.hold(target.pose());
    Pose here = robot.follower.pose();
    status =
        String.format(
            Locale.ROOT,
            "arrived at %s: %.1f in, %.1f deg off",
            target.name(),
            here.distance(target.pose()),
            Math.toDegrees(
                Math.abs(AngleUnit.normalizeRadians(target.pose().heading() - here.heading()))));
    if (tourIndex >= 0) {
      tourLog.add(status);
      dwellUntilMs = System.currentTimeMillis() + tourDwellMs;
    }
  }

  private void startTour() {
    tourLog.clear();
    tourIndex = 0;
    driveTo(stops.get(0));
    if (driving == null) {
      tourLog.add(status); // No route to the first stop: it is skipped like any other.
    }
  }

  /** After the dwell at a stop, on to the next; a stop with no route is skipped, not driven. */
  private void advanceTour() {
    if (tourIndex < 0 || driving != null || System.currentTimeMillis() < dwellUntilMs) {
      return;
    }
    tourIndex++;
    if (tourIndex >= stops.size()) {
      tourIndex = -1;
      status = "tour complete";
      return;
    }
    driveTo(stops.get(tourIndex));
    if (driving == null) {
      tourLog.add(status);
      dwellUntilMs = 0; // Skipped: try the next stop on the next loop.
    }
  }

  private void driveTo(Stop stop) {
    Pose here = robot.follower.pose();
    Pose destination = robot.sentinel.wallStandoff(stop.pose());
    long t0 = System.nanoTime();
    // Not held to the alliance side: the tour visits both alliances' poses.
    List<Pose> planned = robot.sentinel.routeAround(here, destination, false);
    long planMs = (System.nanoTime() - t0) / 1_000_000L;
    target = new Stop(stop.name(), destination);
    routeStart = here;
    route = planned;
    dwellUntilMs = 0;
    if (planned.isEmpty()) {
      status = "NO CLEAR ROUTE to " + stop.name() + " (it would hit the HIVE or a wall): skipped";
      return;
    }
    CommandBuilder legs =
        PathUtil.followAround(robot.follower, robot.sentinel, here, planned, false);
    legs.schedule();
    driving = legs;
    status =
        String.format(
            Locale.ROOT,
            "driving to %s: %d leg(s), planned in %d ms",
            stop.name(),
            planned.size(),
            planMs);
  }

  private void stopHere() {
    CommandBuilder current = driving;
    if (current != null) {
      current.cancel();
      driving = null;
    }
    tourIndex = -1;
    robot.follower.hold(robot.follower.pose());
    status = "stopped: holding here";
  }

  private void show(boolean running) {
    telemetry.addLine(
        running
            ? "X: tour all | D-pad: pick, A: go | Y: near reset corner | B: stop | BACK: zero odometry"
            : "Reset corner: back on the audience wall, left side on the RED wall, facing the rear.");
    if (tourIndex >= 0) {
      telemetry.addData("Tour", "stop %d of %d", tourIndex + 1, stops.size());
    }
    for (int i = 0; i < stops.size(); i++) {
      Stop s = stops.get(i);
      Pose destination = robot.sentinel.wallStandoff(s.pose());
      telemetry.addLine(
          String.format(
              Locale.ROOT,
              "%s %-13s (%.1f, %.1f, %.0f deg)",
              i == selected ? ">" : " ",
              s.name(),
              destination.x(),
              destination.y(),
              Math.toDegrees(destination.heading())));
    }
    telemetry.addData("Status", status);
    Pose here = robot.follower.pose();
    telemetry.addData(
        "Pose", "(%.1f, %.1f, %.0f deg)", here.x(), here.y(), Math.toDegrees(here.heading()));
    StringBuilder legs = new StringBuilder();
    for (Pose p : route) {
      legs.append(String.format(Locale.ROOT, " -> (%.0f, %.0f)", p.x(), p.y()));
    }
    telemetry.addData("Route", route.isEmpty() ? "none" : legs.toString());
    for (String line : tourLog) {
      telemetry.addLine("  " + line);
    }
    telemetry.update();
  }
}
