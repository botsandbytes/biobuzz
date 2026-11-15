package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

public class IntakeCapacityTest {
  @Test
  public void knownFullPreloadCannotBecomeUnknownJustByRequestingCollection() {
    HardwareMap map = mock(HardwareMap.class);
    DcMotorEx motor = mock(DcMotorEx.class);
    CRServo transfer = mock(CRServo.class);
    config.inventory.entry_sensor_enabled = false;
    when(map.get(DcMotorEx.class, "rightIntake")).thenReturn(motor);
    when(map.get(CRServo.class, config.intake.transfer_servo_name)).thenReturn(transfer);
    Intake intake = new Intake(map, () -> 0);
    verify(map).get(DcMotorEx.class, "rightIntake");
    verify(map).get(CRServo.class, config.intake.transfer_servo_name);
    verifyNoMoreInteractions(map);
    intake.inventory().set(config.inventory.capacity_count);
    intake.run(1.0);
    verify(motor, never()).setPower(anyDouble());
    verifyNoInteractions(transfer);
    intake.periodic();
    assertTrue(intake.isBlockedByCapacity());
    assertTrue(intake.inventory().isKnown());
    verify(motor).setPower(0.0);
    verify(transfer).setPower(0.0);
    intake.inventory().onShot();
    intake.periodic();
    assertFalse(intake.isBlockedByCapacity());
    assertFalse(intake.inventory().isKnown());
    verify(motor).setPower(1.0);
    verify(transfer, times(2)).setPower(0.0);
  }

  @Test
  public void fullInventoryStillAllowsFeedingAndEjectingThroughTheSingleWriter() {
    HardwareMap map = mock(HardwareMap.class);
    DcMotorEx motor = mock(DcMotorEx.class);
    CRServo transfer = mock(CRServo.class);
    config.inventory.entry_sensor_enabled = false;
    when(map.get(DcMotorEx.class, "rightIntake")).thenReturn(motor);
    when(map.get(CRServo.class, config.intake.transfer_servo_name)).thenReturn(transfer);
    Intake intake = new Intake(map, () -> 0);
    intake.inventory().set(config.inventory.capacity_count);

    intake.feed(0.7);
    verify(motor, never()).setPower(anyDouble());
    verifyNoInteractions(transfer);
    intake.periodic();
    assertFalse(intake.isBlockedByCapacity());
    assertEquals(Intake.Mode.FEED, intake.mode());
    verify(motor).setPower(0.7);
    verify(transfer).setPower(0);

    clearInvocations(motor, transfer);
    intake.eject(0.6);
    verifyNoInteractions(motor, transfer);
    intake.periodic();
    assertFalse(intake.isBlockedByCapacity());
    assertEquals(Intake.Mode.EJECT, intake.mode());
    verify(motor).setPower(-0.6);
    verify(transfer).setPower(0);

    clearInvocations(motor, transfer);
    intake.stop();
    verifyNoInteractions(motor, transfer);
    intake.periodic();
    assertFalse(intake.isRunning());
    verify(motor).setPower(0.0);
    verify(transfer).setPower(0.0);
  }

  @Test
  void transferFollowsShooterIndependentlyOfIntakeAndManualStopRemainsLatched() {
    HardwareMap map = mock(HardwareMap.class);
    DcMotorEx motor = mock(DcMotorEx.class);
    CRServo transfer = mock(CRServo.class);
    when(map.get(DcMotorEx.class, "rightIntake")).thenReturn(motor);
    when(map.get(CRServo.class, config.intake.transfer_servo_name)).thenReturn(transfer);
    config.intake.transfer_power = 0.4;
    config.intake.transfer_reversed = true;
    AtomicReference<Double> shooterPower = new AtomicReference<>(0.0);
    Intake intake = new Intake(map, shooterPower::get);
    intake.periodic();
    verify(transfer).setPower(0.0);
    clearInvocations(motor, transfer);
    intake.run(0.7);
    intake.periodic();
    verify(transfer).setPower(0.0); // Intake alone must never start the transfer.
    clearInvocations(motor, transfer);
    shooterPower.set(0.6);
    intake.inventory().set(config.inventory.capacity_count);
    intake.periodic();
    verify(motor).setPower(0.0); // Full inventory blocks collection, not shooter transfer.
    verify(transfer).setPower(0.4);
    verify(transfer).setDirection(DcMotorSimple.Direction.REVERSE);

    clearInvocations(motor, transfer);
    intake.stopTransfer();
    intake.feed(0.7);
    verifyNoInteractions(motor, transfer);
    intake.periodic();
    intake.feed(0.7);
    intake.periodic();
    verify(motor, times(2)).setPower(0.7);
    verify(transfer, times(2)).setPower(0.0);

    clearInvocations(motor, transfer);
    intake.resumeTransfer();
    verifyNoInteractions(motor, transfer);
    intake.periodic();
    verify(transfer).setPower(0.4);
    clearInvocations(motor, transfer);
    intake.stop();
    intake.resumeTransfer();
    intake.periodic();
    verify(motor).setPower(0.0);
    verify(transfer).setPower(0.4); // Shooter still runs even though the intake stopped.
    clearInvocations(motor, transfer);
    intake.eject(0.6);
    intake.periodic();
    verify(transfer).setPower(0.4); // Eject must not reverse the shooter's transfer.
    for (double off : new double[] {0, -0.5, Double.NaN, Double.POSITIVE_INFINITY}) {
      clearInvocations(motor, transfer);
      shooterPower.set(off);
      intake.resumeTransfer();
      intake.periodic();
      verify(transfer).setPower(0.0);
    }
  }
}
