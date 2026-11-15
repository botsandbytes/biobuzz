package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class FlowerFlapTest {
  private final HardwareMap hw = mock(HardwareMap.class);
  private final ServoImplEx servo = mock(ServoImplEx.class);

  private FlowerFlap build() {
    when(hw.get(ServoImplEx.class, "flapservo")).thenReturn(servo);
    return new FlowerFlap(hw, true);
  }

  @AfterEach
  void resetScheduler() {
    Scheduler.reset();
  }

  @Test
  void firstPressLowers180DegreesSecondReturnsAndOnlyPeriodicWrites() {
    config.flap.servo_range_deg = 360;
    config.flap.up_angle_deg = 45;
    config.flap.down_angle_deg = 225;
    var flap = build();
    verify(hw).get(ServoImplEx.class, "flapservo");
    verify(servo, never()).setPosition(anyDouble());
    verify(servo, never()).setPwmEnable();
    flap.periodic();
    verify(servo).setPwmDisable();
    verify(servo, never()).setPosition(anyDouble());
    clearInvocations(servo);
    var toggle = flap.toggleCommand();
    toggle.schedule();
    Scheduler.execute();
    assertEquals(FlowerFlap.Target.DOWN, flap.target());
    verifyNoInteractions(servo);
    flap.periodic();
    verify(servo).setPosition(0.625); // This test's 225-degree target / 360-degree sweep.
    verify(servo).setPwmEnable();
    verify(servo).setDirection(Servo.Direction.FORWARD);
    clearInvocations(servo);
    toggle.schedule();
    Scheduler.execute();
    assertEquals(FlowerFlap.Target.UP, flap.target());
    verifyNoInteractions(servo);
    flap.periodic();
    verify(servo).setPosition(0.125); // Return to 45 degrees: 180 degrees of travel.
    clearInvocations(servo);
    flap.stop();
    verifyNoInteractions(servo);
    flap.periodic();
    verify(servo).setPwmDisable();
    verify(servo, never()).setPosition(anyDouble());
  }

  @Test
  void positionTelemetryReportsConfiguredLimitsAndReadbackWithoutWritingHardware() {
    config.flap.servo_range_deg = 360;
    config.flap.up_angle_deg = 324;
    config.flap.down_angle_deg = 36;
    when(servo.getPosition()).thenReturn(0.4);
    var flap = build();
    clearInvocations(servo);
    assertEquals(
        "min 0.100; max 0.900; current 0.400 (SDK command, not measured)", flap.positionStatus());
    verify(servo).getPosition();
    verifyNoMoreInteractions(servo);
    when(servo.getPosition()).thenReturn(Double.NaN);
    assertTrue(flap.positionStatus().contains("current unavailable"));
    assertTrue(new FlowerFlap(hw, false).positionStatus().contains("current unavailable"));
    config.flap.servo_range_deg = 0;
    assertEquals(FlowerFlap.configurationProblem(), flap.positionStatus());
  }

  @Test
  void cancelAndReleaseRememberTheDownCommand() {
    var flap = build();
    flap.setTarget(FlowerFlap.Target.DOWN);
    flap.stop();
    flap.toggle();
    assertEquals(FlowerFlap.Target.UP, flap.target());
    flap.setTarget(FlowerFlap.Target.DOWN);
    flap.setTarget(FlowerFlap.Target.NONE);
    flap.toggle();
    assertEquals(FlowerFlap.Target.UP, flap.target());
  }

  @Test
  void reinitRecoversLastDownPulseAcrossSdkPwmRangeResetAndDirections() {
    config.flap.up_angle_deg = 0;
    config.flap.down_angle_deg = 260;
    config.flap.servo_range_deg = 360;
    config.flap.pwm_lower_us = 500;
    config.flap.pwm_upper_us = 2500;
    for (boolean reversed : new boolean[] {false, true}) {
      config.flap.reversed = reversed;
      // Independent pulse oracle for a 260/360 command at 500..2500 us.
      double pulse = reversed ? 1055.5555555555557 : 1944.4444444444443;
      for (Servo.Direction readDirection : Servo.Direction.values()) {
        double raw = (pulse - 600) / 1800;
        when(servo.getPosition())
            .thenReturn(readDirection == Servo.Direction.REVERSE ? 1 - raw : raw);
        when(servo.getDirection()).thenReturn(readDirection);
        when(servo.getPwmRange()).thenReturn(new PwmControl.PwmRange(600, 2400));
        var flap = build();
        assertEquals(FlowerFlap.Target.NONE, flap.target());
        flap.toggle();
        assertEquals(FlowerFlap.Target.UP, flap.target());
      }
    }
    verify(servo, never()).setPosition(anyDouble());
    verify(servo, never()).setPwmEnable();
  }

  @Test
  void unknownCommandDefaultsDownAndExplicitUpWorksWithoutPositionFeedback() {
    when(servo.getPosition()).thenReturn(Double.NaN);
    var flap = build();
    flap.toggle();
    assertEquals(FlowerFlap.Target.DOWN, flap.target());
    flap.stop();
    flap.upCommand().schedule();
    Scheduler.execute();
    assertEquals(FlowerFlap.Target.UP, flap.target());
    flap.toggle();
    assertEquals(FlowerFlap.Target.DOWN, flap.target());
    verify(servo, never()).setPosition(anyDouble());
  }

  @Test
  void nonTeleOpAndDisabledFlapDoNotRequireOrWriteHardware() {
    var flap = new FlowerFlap(hw, false);
    flap.toggle();
    flap.periodic();
    verifyNoInteractions(hw, servo);
    config.flap.enabled = false;
    flap = new FlowerFlap(hw, true);
    flap.toggle();
    flap.periodic();
    verifyNoInteractions(hw, servo);
  }

  @Test
  void directionAndDifferentProgrammedSweepComeFromConfiguration() {
    config.flap.servo_range_deg = 270;
    config.flap.up_angle_deg = 45;
    config.flap.down_angle_deg = 225;
    config.flap.reversed = true;
    var flap = build();
    flap.toggle();
    flap.periodic();
    verify(servo).setDirection(Servo.Direction.REVERSE);
    verify(servo).setPosition(5.0 / 6);
    clearInvocations(servo);
    flap.toggle();
    flap.periodic();
    verify(servo).setPosition(1.0 / 6);
  }

  @Test
  void invalidAnglesDisableOutputInsteadOfSendingClippedOrNonfiniteCommands() {
    var flap = build();
    flap.toggle();
    for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -1, 361}) {
      config.flap.down_angle_deg = invalid;
      assertNotNull(FlowerFlap.configurationProblem());
      flap.periodic();
    }
    verify(servo, never()).setPosition(anyDouble());
    verify(servo, times(4)).setPwmDisable();
    config.flap.down_angle_deg = 180;
    config.flap.servo_range_deg = 0;
    assertNotNull(FlowerFlap.configurationProblem());
    config.flap.servo_range_deg = 360;
    config.flap.up_angle_deg = 180;
    assertNotNull(FlowerFlap.configurationProblem());
    config.flap.up_angle_deg = 0;
    config.flap.pwm_lower_us = config.flap.pwm_upper_us;
    assertNotNull(FlowerFlap.configurationProblem());
    config.flap.pwm_lower_us = 500;
    config.flap.servo_name = "";
    assertNotNull(FlowerFlap.configurationProblem());
  }

  @Test
  void disablingAnActiveFlapStopsPwmOnTheNextWriterUpdate() {
    var flap = build();
    flap.toggle();
    flap.periodic();
    clearInvocations(servo);
    config.flap.enabled = false;
    flap.periodic();
    verify(servo).setPwmDisable();
    verify(servo, never()).setPosition(anyDouble());
  }
}
