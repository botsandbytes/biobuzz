package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.FlowerFlap;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Operator A toggles the intake. It used to start an intake Command, which shares the intake with
 * aim-and-shoot, so pressing A mid-shot cancelled the shot: flywheel off, and the robot left its
 * aimed heading (TeleOp, 2026-11-14). Now A feeds the shot instead.
 */
class IntakeToggleControlsTest {
  private TeleOp op;
  private CommandBuilder aimAndShoot;

  @BeforeEach
  void build() {
    Scheduler.reset();
    op = new TeleOp();
    op.driver = new Gamepad();
    op.flap = new FlowerFlap(mock(HardwareMap.class), false);
    op.intake = mock(Intake.class);
    op.shooter = mock(Shooter.class);
    op.turret = mock(Turret.class);
    op.follower = mock(Follower.class);
    when(op.follower.pose()).thenReturn(new Pose(30, 30, 0));
    op.casablanca = mock(Casablanca.class);
    op.shotController = mock(ShotController.class);
    when(op.shotController.currentTarget())
        .thenReturn(new CellTarget(HiveCell.A, 50, 50, 60, 0, 0));
    // As the real one does: aim-and-shoot holds the intake while it runs.
    aimAndShoot = Command.build().requiring(op.intake);
    when(op.shotController.fixedShotCommand(anyDouble())).thenReturn(aimAndShoot);
    op.buildCommands();
  }

  @AfterEach
  void resetScheduler() {
    Scheduler.reset();
  }

  /** One TeleOp loop with these operator buttons held (everything else released). */
  private void loop(boolean a, boolean rightTrigger, boolean dpadRight) {
    op.operatorA.update(a);
    op.operatorRightTrigger.update(rightTrigger);
    op.operatorDpadRight.update(dpadRight);
    op.onLoop();
    Scheduler.execute();
  }

  @Test
  void aTogglesCollectingOnAndOff() {
    loop(true, false, false);
    loop(false, false, false);
    verify(op.intake, atLeastOnce()).run(config.teleop.intake_power);
    verify(op.shotController, atLeastOnce()).setOperatorIntake(true);

    clearInvocations(op.intake, op.shotController);
    loop(true, false, false);
    loop(false, false, false);
    verify(op.intake, never()).run(anyDouble());
    verify(op.intake, atLeastOnce()).stop();
    verify(op.shotController, never()).setOperatorIntake(true);
  }

  @Test
  void aDuringAimAndShootFeedsTheShotWithoutCancellingIt() {
    loop(false, true, false);
    loop(false, true, false);
    assertTrue(Scheduler.isScheduled(aimAndShoot));

    clearInvocations(op.intake);
    loop(true, true, false);
    loop(false, true, false);
    assertTrue(Scheduler.isScheduled(aimAndShoot), "pressing A must not cancel the shot");
    verify(op.intake, atLeastOnce()).feed(config.shooter.feed_intake_power);
    verify(op.intake, never()).run(anyDouble());

    // Letting go of the trigger ends the shot; the toggle is still on, so the intake collects.
    clearInvocations(op.intake);
    loop(false, false, false);
    loop(false, false, false);
    assertFalse(Scheduler.isScheduled(aimAndShoot));
    verify(op.intake, atLeastOnce()).run(config.teleop.intake_power);
  }

  @Test
  void ejectBeatsTheToggleAndSwitchesItOff() {
    loop(true, false, false);
    loop(false, false, false);

    clearInvocations(op.intake);
    loop(false, false, true);
    loop(false, false, true);
    verify(op.intake, atLeastOnce()).eject(config.teleop.eject_power);
    verify(op.intake, never()).run(anyDouble());

    clearInvocations(op.intake);
    loop(false, false, false);
    verify(op.intake, never()).run(anyDouble());
    verify(op.intake, atLeastOnce()).stop();
  }
}
