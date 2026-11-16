package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.doubleThat;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.clearInvocations;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * TeleOp's aim-and-shoot (manualShotCommand): drives straight toward or away from the upward CELL
 * to teleop.shot_distance_in, faces it, and feeds once the flywheel is up, the robot is aimed and
 * it has arrived, without a calibrated shot table. AUTO's aimAndShootCommand keeps every gate.
 */
public class ManualShotTest {
  private final Intake intake = mock(Intake.class);
  private final Turret turret = mock(Turret.class);
  private final Follower follower = mock(Follower.class);
  private final Shooter shooter = mock(Shooter.class);
  private final HiveTracker hive = new HiveTracker(3, 1000, 500, () -> 0);
  private Pose pose = new Pose(58, 15, Math.PI / 2);
  private final ShotController controller;

  public ManualShotTest() {
    Scheduler.reset();
    when(shooter.getTargetPower()).thenReturn(0.5);
    when(shooter.getShooterVelocity()).thenReturn(config.shooter.max_rpm * 0.5);
    when(intake.inventory()).thenReturn(new BallInventory(4, true));
    when(turret.isAimed(any())).thenReturn(true);
    when(follower.pose()).thenAnswer(inv -> pose);
    // The solver refuses (as it does with an uncalibrated table); only a manual shot may feed.
    controller =
        new ShotController(
            shooter,
            turret,
            intake,
            () -> pose,
            Velocity::zero,
            null,
            new FieldModel(Alliance.RED),
            hive,
            null,
            (inputs, flight) -> ShotSolution.invalid("test solver"));
  }

  @AfterEach
  void shutdown() {
    controller.shutdown();
    Scheduler.reset();
  }

  private double axisDistance(Pose robot) {
    CellTarget target = controller.currentTarget();
    Pose pivot = Turret.pivotPose(robot);
    return Math.hypot(target.x() - pivot.x(), target.y() - pivot.y());
  }

  private double bearingFromAxis(Pose robot) {
    CellTarget target = controller.currentTarget();
    Pose pivot = Turret.pivotPose(robot);
    return Math.atan2(target.y() - pivot.y(), target.x() - pivot.x());
  }

  private static double angleBetween(double a, double b) {
    return Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
  }

  @Test
  public void tooFarDrivesStraightInToTheShootingDistanceFacingTheCell() {
    Pose here = new Pose(58, 15, Math.PI / 2);
    Pose goal = controller.rangedShotHoldPose(here);
    assertEquals(config.teleop.shot_distance_in, axisDistance(goal), 1e-6);
    assertTrue(angleBetween(goal.heading(), bearingFromAxis(goal)) < 1e-9, "faces the CELL");
    assertEquals(here.x(), goal.x(), 1e-9, "straight toward the CELL, no sideways move");
  }

  @Test
  public void offAxisStaysOnTheLineToTheCell() {
    Pose here = new Pose(40, 30, 0);
    Pose goal = controller.rangedShotHoldPose(here);
    assertEquals(config.teleop.shot_distance_in, axisDistance(goal), 1e-6);
    assertTrue(angleBetween(goal.heading(), bearingFromAxis(goal)) < 1e-9, "faces the CELL");
    // Same bearing from the CELL before and after: it moved along the line, not across it.
    assertTrue(angleBetween(bearingFromAxis(here), bearingFromAxis(goal)) < 1e-9);
  }

  @Test
  public void withinToleranceOnlyTurns() {
    Pose here =
        new Pose(58, 55.06 - config.teleop.shot_distance_in + 0.5 * tolerance(), Math.PI / 4);
    Pose goal = controller.rangedShotHoldPose(here);
    assertEquals(here.x(), goal.x(), 1e-9);
    assertEquals(here.y(), goal.y(), 1e-9);
    assertTrue(angleBetween(goal.heading(), bearingFromAxis(goal)) < 1e-9, "still turns to face");
  }

  private static double tolerance() {
    return config.teleop.shot_position_tolerance_in;
  }

  @Test
  public void aManualShotFeedsWithoutASolutionOnlyOnceItArrives() {
    controller.manualShotCommand(follower).schedule();
    Scheduler.execute();
    clearInvocations(intake);

    controller.periodic(); // still at y = 15, 40 in out
    verify(intake, never()).feed(anyDouble());
    assertEquals("driving to shooting distance", controller.getGateReason());

    pose = new Pose(58, 55.06 - config.teleop.shot_distance_in, Math.PI / 2);
    controller.periodic();
    verify(intake, atLeastOnce()).feed(config.shooter.feed_intake_power);
  }

  @Test
  public void aManualShotFliesTheShotTablesSpeedForWhereItWillShoot() {
    controller.manualShotCommand(follower).schedule(); // drives in from 40 in to the spot
    Scheduler.execute();
    double rpm = ShotTable.fromConfig(BallType.POLLEN).lookup(config.teleop.shot_distance_in).rpm();
    double expected = rpm / config.shooter.max_rpm;
    verify(shooter).setTargetPower(doubleThat(power -> Math.abs(power - expected) < 1e-6));
  }

  @Test
  public void theFixedShotFeedsAtItsPowerOnceUpToSpeedWithoutDrivingOrAiming() {
    when(turret.isAimed(any())).thenReturn(false); // a fixed shot does not check the aim
    controller.fixedShotCommand(0.5).schedule();
    Scheduler.execute();
    verify(shooter).setTargetPower(0.5);
    controller.periodic();
    verify(intake, atLeastOnce()).feed(config.shooter.feed_intake_power);
    verify(follower, never()).hold(any());
  }

  @Test
  public void autosShotStillNeedsAValidSolution() {
    pose = new Pose(58, 27, Math.PI / 2);
    controller.aimAndShootCommand(follower).schedule();
    Scheduler.execute();
    for (int i = 0; i < 3; i++) {
      controller.periodic();
    }
    verify(intake, never()).feed(anyDouble());
  }

  @Test
  public void aFixedVolleyWaitsForItsPoseAndStopsFeedingWhenTheCellChanges() {
    Pose spot = new Pose(58, 27, Math.PI / 2);
    controller.fixedVolleyCommand(follower, spot, HiveCell.A, 0.5, 4).schedule();
    tickVolley();
    verify(follower).hold(spot);
    verify(shooter, never()).setTargetPower(0.5);
    verify(intake, never()).feed(anyDouble());

    pose = new Pose(58, 27, 0);
    tickVolley();
    verify(shooter, never()).setTargetPower(0.5);

    pose = spot;
    tickVolley();
    verify(shooter).setTargetPower(0.5);
    verify(intake, atLeastOnce()).feed(config.shooter.feed_intake_power);
    clearInvocations(intake, shooter);

    hive.declareUp(HiveCell.B);
    Scheduler.execute();
    controller.periodic();
    verify(shooter).setTargetPower(0.0);
    verify(intake, never()).feed(anyDouble());
  }

  private void tickVolley() {
    for (int i = 0; i < 5; i++) {
      Scheduler.execute();
      controller.periodic();
    }
  }
}
