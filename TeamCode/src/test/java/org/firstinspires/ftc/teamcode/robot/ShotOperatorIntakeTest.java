package org.firstinspires.ftc.teamcode.robot;

import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.clearInvocations;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * Who owns the intake while a shot is active and its gate is closed. By default the shot holds the
 * intake off, so nothing reaches the flywheel before every gate passes. In TeleOp, an operator who
 * has the intake on is feeding by hand at their own call (the shot table may not be calibrated), so
 * a closed gate must leave their intake alone; a lost pose still stops it.
 */
public class ShotOperatorIntakeTest {
  private final Intake intake = mock(Intake.class);
  private Pose pose = new Pose(58, 27, Math.PI / 2);
  private final ShotController controller;

  public ShotOperatorIntakeTest() {
    Shooter shooter = mock(Shooter.class);
    when(shooter.getTargetPower()).thenReturn(0.5);
    when(shooter.getShooterVelocity()).thenReturn(config.shooter.max_rpm * 0.5);
    when(intake.inventory()).thenReturn(new BallInventory(4, true));
    // The mocked turret never reports aimed and the solver refuses: the gate stays closed.
    controller =
        new ShotController(
            shooter,
            mock(Turret.class),
            intake,
            () -> pose,
            Velocity::zero,
            null,
            new FieldModel(Alliance.RED),
            new HiveTracker(3, 1000, 500, () -> 0),
            null,
            (inputs, flight) -> ShotSolution.invalid("test solver"));
    controller.startShot(0.5, true);
    clearInvocations(intake);
  }

  @AfterEach
  void shutdown() {
    controller.shutdown();
  }

  @Test
  public void aClosedGateHoldsTheIntakeOff() {
    controller.periodic();
    verify(intake, atLeastOnce()).stop();
    verify(intake, never()).feed(anyDouble());
  }

  @Test
  public void aClosedGateLeavesTheOperatorsIntakeAlone() {
    controller.setOperatorIntake(true);
    for (int i = 0; i < 5; i++) {
      controller.periodic();
    }
    verify(intake, never()).stop();
    verify(intake, never()).feed(anyDouble());
  }

  @Test
  public void handingTheIntakeBackLetsTheGateHoldItOffAgain() {
    controller.setOperatorIntake(true);
    controller.periodic();
    controller.setOperatorIntake(false);
    controller.periodic();
    verify(intake, atLeastOnce()).stop();
  }

  @Test
  public void aLostPoseStopsTheIntakeEvenForTheOperator() {
    controller.setOperatorIntake(true);
    pose = new Pose(Double.NaN, 27, Math.PI / 2);
    controller.periodic();
    verify(intake, atLeastOnce()).stop();
  }
}
