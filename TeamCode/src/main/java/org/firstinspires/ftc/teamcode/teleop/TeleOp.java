package org.firstinspires.ftc.teamcode.teleop;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.Nullable;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.auto.PathUtil;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.FlowerFlap;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;

/**
 * BIOBUZZ TeleOp. Controls, in short:
 *
 * <p>Driver: sticks drive. Directions are as the driver sees the field from their alliance wall. LT
 * / RT make the CELL on the driver's left / right the goal (the camera is off), drive around the
 * HIVE to that CELL's shooting spot and hold it, and lock the heading onto the CELL. The D-pad
 * drives around the HIVE to the FLOWER that way and holds its pickup pose (up / down: the FLOWER on
 * the far / their own alliance wall; left / right: the one on the wall on that side), only while
 * {@code teleop.flower_hold_enabled}. Y holds the LOADING ZONE park pose. X cancels every drive and
 * the heading lock and hands the sticks back; any stick also cancels a drive, and steering releases
 * the lock. A toggles the flower flap. B stops only the intake transfer servo. BACK (also during
 * INIT) resets odometry to {@code teleop.poses.<alliance>.reset} with the robot pushed into the
 * corner of the audience wall and the RED alliance wall (back on the audience wall).
 *
 * <p>Operator: right trigger (hold) shoots from the one fixed spot (driver LT/RT drive there):
 * flywheel at {@code teleop.manual_rev_power}, intake as soon as it is at speed; no driving, aiming
 * or shot table. A toggles the intake (it feeds while the shooter runs, so A during aim-and-shoot
 * feeds straight away, before the flywheel is up), B stops it, dpad right (hold) eject, Y (hold)
 * turret aim, X (hold) rev with transfer, dpad up raises the flap, left trigger cancels everything,
 * dpad down (hold) vision relocalize, left bumper records a TIP the camera missed, back undoes one,
 * right bumper toggles POLLEN/NECTAR shot tables (only while {@code
 * shooter.shot_table.nectar_enabled}), left stick button marks the robot empty. Operator D-pad left
 * toggles the flower flap DOWN/UP from its last command; right-stick click explicitly raises it.
 */
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "!")
// Commands are assigned by buildCommands(), which TeleOpBase.init() calls.
@SuppressWarnings("NullAway.Init")
public class TeleOp extends TeleOpBase {

  private Command parkCommand;

  /**
   * The drive around the HIVE to a shooting spot (driver LT/RT) or a FLOWER (driver D-pad), or
   * null; holds its destination until cancelled.
   */
  private @Nullable Command trip;

  private Command flapToggleCommand;
  private Command flapUpCommand;
  private Command aimAndShootCommand;
  private Command aimCommand;
  private Command manualRevCommand;

  /** Latched state of the driver's face-the-upward-CELL heading lock; toggled by driver X. */
  private boolean cellHeadingLockEngaged = false;

  /** Operator A's intake toggle. B, eject and cancel-everything turn it off. */
  private boolean intakeOn = false;

  @Override
  protected void buildCommands() {
    parkCommand =
        Command.build().setStart(() -> holdPoseIfClear(profile.parkPose())).requiring(follower);

    flapToggleCommand = flap.toggleCommand();
    flapUpCommand = flap.upCommand();

    // One fixed spot per CELL: driver LT/RT drive to it and hold; this only shoots. Same power
    // as operator X,
    // so one value (teleop.manual_rev_power) is tuned for that spot. The dynamic shot
    // (shotController.manualShotCommand: drive to a distance, shot-table speed) is not bound.
    aimAndShootCommand = shotController.fixedShotCommand(config.teleop.manual_rev_power);

    // Turret only, no flywheel. Aim (operator Y) and rev (operator X) are separate commands over
    // disjoint subsystems so holding both composes into aim-and-spin-up.
    aimCommand =
        Command.build()
            .setStart(() -> turret.setAimMode(Turret.AimMode.AIM_AT_TARGET))
            .setEnd(
                interrupted -> {
                  turret.setHoldAngle(0.0);
                  turret.setAimMode(Turret.AimMode.HOLD);
                })
            .requiring(turret);

    // Shooter target also drives the transfer; the intake roller stays with applyIntake.
    manualRevCommand =
        Command.build()
            .setStart(() -> shooter.setTargetPower(config.teleop.manual_rev_power))
            .setEnd(interrupted -> shooter.setTargetPower(0.0))
            .requiring(shooter);
  }

  @Override
  protected boolean hasActiveDriveOverride() {
    Command current = trip;
    return Scheduler.isScheduled(parkCommand)
        || (current != null && Scheduler.isScheduled(current));
  }

  /**
   * Drives around the HIVE ({@code Sentinel.routeAround}, on the alliance side when it can) to
   * {@code destination} and holds it until driver X or a stick cancels it. Not a straight hold:
   * with footprint safety off one would drive through the HIVE frame from the other side of it.
   */
  private void driveAround(Pose destination, String name) {
    driveAround(destination, name, holdAt(destination));
  }

  /** Holds {@code pose} until the trip is cancelled. */
  private Command holdAt(Pose pose) {
    return Command.build().setStart(() -> follower.hold(pose)).requiring(follower);
  }

  /**
   * Drives around the HIVE to {@code teleop.flower_flap_lead_in} short of the FLOWER pickup pose
   * (backed off along its heading), lowers the flower flap, waits {@code
   * teleop.flower_flap_settle_ms} for it to come down, then drives the last stretch straight in and
   * holds there. The flap is only target state here, so the trip never takes the flap from its
   * toggle commands.
   */
  private void driveToFlower(Pose flower) {
    double lead = config.teleop.flower_flap_lead_in;
    Pose staging =
        new Pose(
            flower.x() - lead * Math.cos(flower.heading()),
            flower.y() - lead * Math.sin(flower.heading()),
            flower.heading());
    driveAround(
        staging,
        "the FLOWER",
        sequential(
            instant(() -> flap.setTarget(FlowerFlap.Target.DOWN)),
            waitMs(config.teleop.flower_flap_settle_ms),
            PathUtil.followRoute(follower, staging, List.of(flower)),
            holdAt(flower)));
  }

  /** Drives around the HIVE to {@code destination}, then runs {@code arrival}. */
  private void driveAround(Pose destination, String name, Command arrival) {
    cancelTrip();
    Pose here = follower.pose();
    if (!Double.isFinite(here.x())
        || !Double.isFinite(here.y())
        || !Double.isFinite(here.heading())) {
      setDriveAssistStatus("pose lost: drive to " + name);
      return;
    }
    boolean allianceSide = true;
    List<Pose> route = sentinel.routeAround(here, destination, true);
    if (route.isEmpty()) {
      allianceSide = false;
      route = sentinel.routeAround(here, destination, false);
    }
    if (route.isEmpty()) {
      setDriveAssistStatus("no clear route to " + name + ": drive there");
      return;
    }
    Command next =
        sequential(PathUtil.followAround(follower, sentinel, here, route, allianceSide), arrival);
    next.schedule();
    trip = next;
    setDriveAssistStatus(String.format(Locale.ROOT, "to %s, %d leg(s)", name, route.size()));
  }

  private void cancelTrip() {
    Command current = trip;
    if (current != null) {
      current.cancel();
      trip = null;
    }
  }

  /** Cancels the park hold and any drive to a spot or FLOWER; the sticks drive again next loop. */
  private void cancelDrives() {
    parkCommand.cancel();
    cancelTrip();
  }

  @Override
  protected void onLoop() {
    handleOperator();
    handleDriver();
  }

  private void handleOperator() {
    if (operatorDpadUp.onTrue() || operatorRightStick.onTrue()) {
      flapUpCommand.schedule();
    } else if (operatorDpadLeft.onTrue()) {
      flapToggleCommand.schedule();
    }
    if (operatorA.onTrue()) {
      intakeOn = !intakeOn;
    }
    // Eject (hold dpad right) spits out a fifth SCORING ELEMENT (G407) or opponent NECTAR (G408).
    // It switches the toggle off, so letting go does not pull the element straight back in.
    if (operatorB.onTrue() || operatorDpadRight.onTrue()) {
      intakeOn = false;
    }

    if (operatorRightTrigger.onTrue()) {
      intake.resumeTransfer();
      aimAndShootCommand.schedule();
    }
    if (operatorRightTrigger.onFalse()) {
      aimAndShootCommand.cancel();
    }

    if (operatorY.onTrue()) {
      aimCommand.schedule();
    }
    if (operatorY.onFalse()) {
      aimCommand.cancel();
    }

    if (operatorX.onTrue()) {
      intake.resumeTransfer();
      manualRevCommand.schedule();
    }
    if (operatorX.onFalse()) {
      manualRevCommand.cancel();
    }

    if (operatorLeftTrigger.onTrue()) {
      aimAndShootCommand.cancel();
      aimCommand.cancel();
      manualRevCommand.cancel();
      intakeOn = false;
      flapToggleCommand.cancel();
      flapUpCommand.cancel();
      flap.stop();
    }

    // HIVE bookkeeping the camera may have missed. Each press changes the tracker's epoch, which
    // invalidates any shot solved against the previous upward CELL.
    if (operatorLeftBumper.onTrue()) {
      hive.recordTip();
    }
    if (operatorBack.onTrue()) {
      hive.undoTip();
    }

    if (operatorRightBumper.onTrue()) {
      shotController.setBallType(shotController.getBallType().toggled());
    }

    if (operatorLeftStick.onTrue()) {
      intake.inventory().set(0);
    }

    applyIntake();
  }

  /**
   * The operator's intake target, set every loop after the buttons. Not a Command: one would share
   * the intake with aim-and-shoot, and starting it cancelled the shot (flywheel off, and the robot
   * left its aimed heading). A held eject beats the toggle. While the shooter is commanded
   * (aim-and-shoot or rev) the toggle feeds, which skips the G407 collect cap because feeding only
   * empties the robot; otherwise it collects, capped. An open shot gate feeds whatever this says.
   */
  private void applyIntake() {
    boolean ejecting = operatorDpadRight.state();
    boolean shooting =
        Scheduler.isScheduled(aimAndShootCommand) || Scheduler.isScheduled(manualRevCommand);
    if (ejecting) {
      intake.eject(config.teleop.eject_power);
    } else if (intakeOn && shooting) {
      intake.feed(config.shooter.feed_intake_power);
    } else if (intakeOn) {
      intake.run(config.teleop.intake_power);
    } else {
      intake.stop();
    }
    shotController.setOperatorIntake(ejecting || intakeOn);
  }

  private void handleDriver() {
    if (driverB.onTrue()) {
      intake.stopTransfer();
    }
    if (driverA.onTrue()) {
      flapToggleCommand.schedule();
    }

    // Driver LT / RT: the CELL on the driver's left / right is the goal; the robot drives to that
    // CELL's shooting spot around the HIVE, and the heading lock faces it afterwards. With the
    // camera off nothing else says which CELL is up, so this tells the HIVE tracker (the only shot
    // target, G417); it keeps the TIP count. The lock keeps the chassis facing the upward CELL
    // while the driver translates. It is republished every loop so the bearing follows the robot
    // and flips to the other CELL on a TIP. Deliberately the raw bearing, not the lead-compensated
    // azimuth: lead is proportional to robot velocity, so the lock would chase its own motion.
    if (driverLeftTrigger.onTrue() || driverRightTrigger.onTrue()) {
      HiveCell left = cellOnDriversLeft(field, profile.alliance());
      HiveCell goal = driverLeftTrigger.onTrue() ? left : left.other();
      hive.declareUp(goal);
      cellHeadingLockEngaged = true;
      parkCommand.cancel();
      driveAround(profile.shootPose(goal), "the CELL " + goal + " spot");
    }

    // Driver D-pad: the FLOWER that way, as the driver sees the field. Facing a CELL is no use
    // there, so the lock lets go.
    double flowerBearing = dpadBearing();
    if (!Double.isNaN(flowerBearing) && config.teleop.flower_hold_enabled) {
      cellHeadingLockEngaged = false;
      parkCommand.cancel();
      driveToFlower(flowerToward(field, profile.flowerPose(), flowerBearing));
    }

    if (driverY.onTrue()) {
      cellHeadingLockEngaged = false;
      cancelTrip();
      parkCommand.schedule();
    }

    // Driver X hands everything back to the sticks: every drive and the heading lock.
    if (driverX.onTrue()) {
      cancelDrives();
      cellHeadingLockEngaged = false;
      setDriveAssistStatus("cancelled (driver X)");
    }

    // Steering releases the lock, so a latched mode can never leave the driver unable to turn.
    if (Math.abs(driver.right_stick_x) > Casablanca.headingLockIntentThreshold) {
      cellHeadingLockEngaged = false;
    }

    Pose pose = follower.pose();
    CellTarget target = shotController.currentTarget();
    casablanca.setTargetHeadingLock(
        Turret.alignPose(pose.x(), pose.y(), target.x(), target.y()).heading(),
        cellHeadingLockEngaged);

    boolean driverStickDeflected =
        Math.abs(driver.left_stick_y) > 0.2
            || Math.abs(driver.left_stick_x) > 0.2
            || Math.abs(driver.right_stick_x) > 0.2;

    if (driverStickDeflected) {
      cancelDrives();
    }
  }

  /**
   * The driver D-pad direction pressed this loop, counter-clockwise from the driver's forward (up
   * 0, left pi/2, down pi, right -pi/2), or NaN when none was.
   */
  private double dpadBearing() {
    if (driverDpadUp.onTrue()) {
      return 0.0;
    }
    if (driverDpadLeft.onTrue()) {
      return Math.PI / 2;
    }
    if (driverDpadDown.onTrue()) {
      return Math.PI;
    }
    if (driverDpadRight.onTrue()) {
      return -Math.PI / 2;
    }
    return Double.NaN;
  }

  /**
   * The pickup pose of the FLOWER {@code bearingRad} counter-clockwise from the driver's forward,
   * as the driver sees the field from their alliance wall: of the four FLOWER approaches ({@link
   * FieldModel#flowerApproaches} of {@code flowerApproach}), the one farthest from the field centre
   * that way. Up (0) is the FLOWER on the far alliance wall, down (pi) the one on the driver's own.
   */
  static Pose flowerToward(FieldModel field, Pose flowerApproach, double bearingRad) {
    double way = driversForwardRad(field.alliance()) + bearingRad;
    double dx = Math.cos(way);
    double dy = Math.sin(way);
    Pose best = flowerApproach;
    double bestReach = Double.NEGATIVE_INFINITY;
    for (Pose approach : field.flowerApproaches(flowerApproach)) {
      double reach = (approach.x() - field.center()) * dx + (approach.y() - field.center()) * dy;
      if (reach > bestReach) {
        best = approach;
        bestReach = reach;
      }
    }
    return best;
  }

  /** The driver's forward on the field: away from their alliance wall, +x for RED, -x for BLUE. */
  private static double driversForwardRad(Alliance alliance) {
    return alliance == Alliance.BLUE ? Math.PI : 0.0;
  }

  /**
   * The CELL of this alliance's HIVE on the driver's left, seen from their alliance wall
   * (stick-forward is +x for RED, -x for BLUE). CELL B for both alliances: the field is a
   * half-turn, so each driver sees their own HIVE the same way round.
   */
  static HiveCell cellOnDriversLeft(FieldModel field, Alliance alliance) {
    double forward = driversForwardRad(alliance);
    double leftX = -Math.sin(forward);
    double leftY = Math.cos(forward);
    FieldModel.Opening a = field.upOpening(HiveCell.A);
    FieldModel.Opening b = field.upOpening(HiveCell.B);
    return b.x() * leftX + b.y() * leftY > a.x() * leftX + a.y() * leftY ? HiveCell.B : HiveCell.A;
  }
}
