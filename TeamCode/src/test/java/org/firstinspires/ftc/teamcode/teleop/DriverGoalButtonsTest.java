package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.FlowerFlap;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;

/**
 * Driver buttons, as the driver sees the field from their alliance wall. LT / RT: with the camera
 * off, they say which CELL is the goal (the CELL on the driver's left / right), drive to its spot
 * and turn the heading lock onto it. The D-pad drives to the FLOWER that way, Y parks, X cancels
 * every drive and the lock, and A toggles the flower flap.
 */
class DriverGoalButtonsTest {
  private static final double UP = 0.0;
  private static final double LEFT = Math.PI / 2;
  private static final double DOWN = Math.PI;
  private static final double RIGHT = -Math.PI / 2;

  @AfterEach
  void resetScheduler() {
    Scheduler.reset();
  }

  /** RED drives toward +x, so +y (the rear CELL, B) is on the left; BLUE is the half-turn. */
  @Test
  void cellBIsOnTheLeftOfBothAlliancesDrivers() {
    for (Alliance alliance : Alliance.values()) {
      assertEquals(
          HiveCell.B, TeleOp.cellOnDriversLeft(new FieldModel(alliance), alliance), "" + alliance);
    }
  }

  /** The index of the FLOWER (RED's two, then BLUE's two) nearest {@code pose}. */
  private static int nearestFlower(Pose pose) {
    List<Coordinate> flowers = new ArrayList<>(new FieldModel(Alliance.RED).allianceSideFlowers());
    flowers.addAll(new FieldModel(Alliance.BLUE).allianceSideFlowers());
    Coordinate at = new Coordinate(pose.x(), pose.y());
    int nearest = 0;
    for (int i = 1; i < flowers.size(); i++) {
      if (flowers.get(i).distance(at) < flowers.get(nearest).distance(at)) {
        nearest = i;
      }
    }
    return nearest;
  }

  /**
   * The RED driver faces +x with the rear wall (+y) on their left: up is the FLOWER on BLUE's
   * alliance wall, down the one on RED's own, left the rear-wall one, right the audience-wall one.
   */
  @Test
  void theDpadPicksTheFlowerThatWayAsTheRedDriverSeesIt() {
    FieldModel red = new FieldModel(Alliance.RED);
    Pose own = config.loadMatchProfile(Alliance.RED).flowerPose();
    // allianceSideFlowers: side (alliance) wall first, then rear / front wall; RED's, then BLUE's.
    assertEquals(2, nearestFlower(TeleOp.flowerToward(red, own, UP)), "up: BLUE's wall");
    assertEquals(1, nearestFlower(TeleOp.flowerToward(red, own, LEFT)), "left: rear wall");
    assertEquals(0, nearestFlower(TeleOp.flowerToward(red, own, DOWN)), "down: RED's wall");
    assertEquals(3, nearestFlower(TeleOp.flowerToward(red, own, RIGHT)), "right: audience wall");
    Pose down = TeleOp.flowerToward(red, own, DOWN);
    assertEquals(own.x(), down.x(), 1e-9);
    assertEquals(own.y(), down.y(), 1e-9);
  }

  /** BLUE's driver sees the field half-turned, so each D-pad press is RED's answer half-turned. */
  @Test
  void blueDriversDpadIsTheHalfTurnOfRedDrivers() {
    FieldModel red = new FieldModel(Alliance.RED);
    FieldModel blue = new FieldModel(Alliance.BLUE);
    Pose redOwn = config.loadMatchProfile(Alliance.RED).flowerPose();
    Pose blueOwn = config.loadMatchProfile(Alliance.BLUE).flowerPose();
    for (double bearing : new double[] {UP, LEFT, DOWN, RIGHT}) {
      Pose expected = blue.fromRedFrame(TeleOp.flowerToward(red, redOwn, bearing));
      Pose got = TeleOp.flowerToward(blue, blueOwn, bearing);
      assertEquals(expected.x(), got.x(), 1e-9, "bearing " + bearing);
      assertEquals(expected.y(), got.y(), 1e-9, "bearing " + bearing);
    }
  }

  private static TeleOp newTeleOp(Pose at) {
    Scheduler.reset();
    TeleOp op = new TeleOp();
    op.driver = new Gamepad();
    op.flap = new FlowerFlap(mock(HardwareMap.class), false);
    op.intake = mock(Intake.class);
    op.shooter = mock(Shooter.class);
    op.turret = mock(Turret.class);
    op.follower = mock(Follower.class);
    when(op.follower.pose()).thenReturn(at);
    op.casablanca = mock(Casablanca.class);
    op.shotController = mock(ShotController.class);
    when(op.shotController.currentTarget())
        .thenReturn(new CellTarget(HiveCell.A, 58, 55, 60, 0, 0));
    when(op.shotController.fixedShotCommand(anyDouble())).thenReturn(Command.build());
    op.field = new FieldModel(Alliance.RED);
    op.sentinel = new Sentinel(op.field);
    op.profile = config.loadMatchProfile(Alliance.RED);
    op.hive = new HiveTracker(3, 1000, 500, () -> 0);
    op.buildCommands();
    return op;
  }

  /** One loop with {@code button} pressed, then one with it released. */
  private static void tap(TeleOp op, TeleOpBase.ButtonTracker button) {
    button.update(true);
    op.onLoop();
    Scheduler.execute();
    button.update(false);
  }

  @Test
  void triggersPickTheGoalAndLockTheHeadingOntoIt() {
    TeleOp op = newTeleOp(new Pose(58, 27, Math.PI / 2));

    // At shoot_a; LT: the left CELL (B) is the goal, and the robot drives to its spot.
    tap(op, op.driverLeftTrigger);
    assertEquals(HiveCell.B, op.hive.upCell());
    assertEquals(0, op.hive.tips(), "choosing a goal is not a TIP");
    verify(op.casablanca, atLeastOnce()).setTargetHeadingLock(anyDouble(), eq(true));
    assertTrue(op.hasActiveDriveOverride(), "driving to CELL B's spot");

    // A stick hands the drive back.
    op.driver.left_stick_y = 0.5f;
    op.onLoop();
    Scheduler.execute();
    assertFalse(op.hasActiveDriveOverride(), "the stick cancels the drive to the spot");

    op.driver.left_stick_y = 0f;
    tap(op, op.driverRightTrigger);
    assertEquals(HiveCell.A, op.hive.upCell());
    assertTrue(op.hasActiveDriveOverride(), "holding CELL A's spot (already there)");
  }

  @Test
  void xCancelsEveryDriveAndTheHeadingLock() {
    TeleOp op = newTeleOp(new Pose(58, 27, Math.PI / 2));
    tap(op, op.driverRightTrigger);
    assertTrue(op.hasActiveDriveOverride(), "holding CELL A's spot");

    clearInvocations(op.casablanca);
    tap(op, op.driverX);
    assertFalse(op.hasActiveDriveOverride(), "X cancels the drive to the spot");
    verify(op.casablanca, never()).setTargetHeadingLock(anyDouble(), eq(true));
    verify(op.casablanca).setTargetHeadingLock(anyDouble(), eq(false));

    tap(op, op.driverY);
    assertTrue(op.hasActiveDriveOverride(), "Y holds the park pose");
    tap(op, op.driverX);
    assertFalse(op.hasActiveDriveOverride(), "X cancels the park hold");
  }

  @Test
  void theDpadDrivesToAFlowerOnlyWhileFlowerHoldIsEnabled() {
    TeleOp op = newTeleOp(config.loadMatchProfile(Alliance.RED).resetPose());
    tap(op, op.driverRightTrigger);
    assertTrue(op.hasActiveDriveOverride(), "driving to CELL A's spot");

    clearInvocations(op.casablanca);
    tap(op, op.driverDpadDown);
    assertTrue(op.hasActiveDriveOverride(), "driving to RED's own FLOWER");
    verify(op.casablanca, never()).setTargetHeadingLock(anyDouble(), eq(true));

    tap(op, op.driverX);
    config.teleop.flower_hold_enabled = false;
    tap(op, op.driverDpadDown);
    assertFalse(op.hasActiveDriveOverride(), "the D-pad does nothing while flower hold is off");
  }

  @Test
  void aTogglesTheFlowerFlap() {
    TeleOp op = newTeleOp(new Pose(58, 27, Math.PI / 2));
    assertEquals(FlowerFlap.Target.NONE, op.flap.target(), "never moved before a command");
    tap(op, op.driverA);
    assertEquals(FlowerFlap.Target.DOWN, op.flap.target());
    tap(op, op.driverA);
    assertEquals(FlowerFlap.Target.UP, op.flap.target());
  }
}
