package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.*;
import java.util.List;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.ArgumentCaptor;

class ShooterControlsTest {
  @AfterEach
  void resetScheduler() {
    Scheduler.reset();
  }

  @ParameterizedTest
  @ValueSource(booleans = {false, true})
  @SuppressWarnings("unchecked")
  void operatorXDrivesBothMotorsAndReleaseStopsBoth(boolean recoveryKick) {
    Scheduler.reset();
    config.shooter.use_ftc_pid = false;
    if (!recoveryKick) config.shooter.kick.wheel_tau_sec = 0;
    HardwareMap map = mock(HardwareMap.class);
    DcMotorEx first = mock(DcMotorEx.class);
    DcMotorEx second = mock(DcMotorEx.class);
    DcMotorEx intakeMotor = mock(DcMotorEx.class);
    CRServo transfer = mock(CRServo.class);
    when(map.get(DcMotorEx.class, "shooter")).thenReturn(first);
    when(map.get(DcMotorEx.class, "shooter2")).thenReturn(second);
    when(map.get(DcMotorEx.class, "rightIntake")).thenReturn(intakeMotor);
    when(map.get(CRServo.class, config.intake.transfer_servo_name)).thenReturn(transfer);
    when(first.getMode()).thenReturn(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    when(second.getMode()).thenReturn(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    map.voltageSensor = mock(HardwareMap.DeviceMapping.class);
    VoltageSensor voltage = mock(VoltageSensor.class);
    when(voltage.getVoltage()).thenReturn(12.0);
    when(map.voltageSensor.iterator()).thenAnswer(inv -> List.of(voltage).iterator());

    TeleOp op = new TeleOp();
    op.driver = new Gamepad();
    op.flap = new FlowerFlap(map, false);
    op.shooter = new Shooter(map);
    op.intake = new Intake(map, op.shooter::getTargetPower);
    op.turret = mock(Turret.class);
    op.follower = mock(Follower.class);
    when(op.follower.pose()).thenReturn(new Pose(30, 30, 0));
    op.casablanca = mock(Casablanca.class);
    op.shotController = mock(ShotController.class);
    when(op.shotController.currentTarget())
        .thenReturn(new CellTarget(HiveCell.A, 50, 50, 60, 0, 0));
    when(op.shotController.fixedShotCommand(anyDouble())).thenReturn(Command.build());
    op.buildCommands();
    clearInvocations(first, second, intakeMotor, transfer);

    op.operatorX.update(true);
    op.onLoop();
    Scheduler.execute();
    assertEquals(config.teleop.manual_rev_power, op.shooter.getTargetPower());
    verify(first, never()).setPower(anyDouble());
    verify(second, never()).setPower(anyDouble());
    verify(transfer, never()).setPower(anyDouble());
    op.intake.periodic();
    op.shooter.periodic();
    verify(transfer).setPower(config.intake.transfer_power);
    verify(intakeMotor).setPower(0);
    ArgumentCaptor<Double> a = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> b = ArgumentCaptor.forClass(Double.class);
    verify(first).setPower(a.capture());
    verify(second).setPower(b.capture());
    assertTrue(a.getValue() > 0);
    assertEquals(a.getValue(), b.getValue());

    clearInvocations(first, second, intakeMotor, transfer);
    op.operatorX.update(true);
    op.driverB.update(true);
    op.onLoop();
    Scheduler.execute();
    op.intake.periodic();
    op.shooter.periodic();
    assertEquals(config.teleop.manual_rev_power, op.shooter.getTargetPower());
    verify(transfer).setPower(0); // Independent stop leaves the flywheels running.
    clearInvocations(first, second, intakeMotor, transfer);
    op.driverB.update(false);
    op.operatorA.update(true);
    op.operatorX.update(true);
    op.onLoop();
    Scheduler.execute();
    op.intake.periodic();
    verify(intakeMotor).setPower(config.teleop.intake_power);
    verify(transfer).setPower(0); // Intake A does not clear a transfer stop.

    clearInvocations(first, second, intakeMotor, transfer);
    op.operatorA.update(false);
    op.operatorX.update(false);
    op.onLoop();
    Scheduler.execute();
    op.intake.periodic();
    op.shooter.periodic();
    verify(first).setPower(0);
    verify(second).setPower(0);
    verify(transfer).setPower(0);
    verify(intakeMotor).setPower(config.teleop.intake_power); // Roller remains independent.

    clearInvocations(first, second, intakeMotor, transfer);
    op.operatorX.update(true);
    op.onLoop();
    Scheduler.execute();
    op.intake.periodic();
    op.shooter.periodic();
    verify(transfer).setPower(config.intake.transfer_power); // A fresh X press resumes transfer.

    clearInvocations(first, second, intakeMotor, transfer);
    op.operatorX.update(true);
    op.operatorLeftTrigger.update(true);
    op.onLoop();
    Scheduler.execute();
    op.intake.periodic();
    op.shooter.periodic();
    verify(transfer).setPower(0);
    verify(intakeMotor).setPower(0);
    verify(first).setPower(0);
    verify(second).setPower(0);
  }
}
