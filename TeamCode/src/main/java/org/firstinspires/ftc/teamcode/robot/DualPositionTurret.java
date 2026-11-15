package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Positional Axons close their own shaft loops. Never send the legacy CR power law to them.
 *
 * <p>Like {@link Turret}, this reads config and hardware, calls {@link TurretLaw}'s {@code axon*}
 * decisions (proven by OpenJML), and writes the servos; it decides nothing itself.
 */
final class DualPositionTurret {
  private final AxonTurretServos servos;
  private final LongSupplier clock;
  private double offset;
  private long lastUpdate = Long.MIN_VALUE;
  private long enabledAt;
  private boolean active;
  private boolean manualReleaseRequired;
  private @Nullable String fault;
  private @Nullable String blocked = "PWM disabled";

  DualPositionTurret(HardwareMap hw, LongSupplier clock) {
    servos = new AxonTurretServos(hw);
    this.clock = clock;
  }

  static double degreesPerPosition() {
    var c = config.turret.dual_position;
    return TurretLaw.axonDegreesPerPosition(
        c.servo_sweep_deg, c.servo_gear_teeth, c.turret_gear_teeth, c.positive_offset_ccw);
  }

  static @Nullable String configurationProblem() {
    var c = config.turret.dual_position;
    double fullScale = config.turret.analog_encoder.full_scale_voltage;
    // Repeats config-docs ranges on purpose: the agent bridge sets config in memory, past the
    // loader's clamp.
    if (!(c.servo_gear_teeth > 0
        && c.turret_gear_teeth > 0
        && c.servo_sweep_deg > 0
        && c.servo_sweep_deg <= 360
        && c.left_volts_per_position > 0
        && Double.isFinite(c.left_volts_per_position)
        && c.right_volts_per_position > 0
        && Double.isFinite(c.right_volts_per_position)
        && TurretLaw.axonFeedbackValid(c.left_reference_volts, fullScale)
        && TurretLaw.axonFeedbackValid(c.right_reference_volts, fullScale)
        && c.manual_position_per_sec > 0
        && c.manual_position_per_sec <= 1
        && c.feedback_startup_sec >= 0
        && c.feedback_startup_sec <= 2
        && c.max_update_gap_sec > 0
        && c.max_update_gap_sec <= 0.5)) {
      return "Invalid dual positional turret calibration or timing";
    }
    return DualAxonTurretTest.configurationProblem();
  }

  static boolean calibrated() {
    return config.turret.dual_position.angle_calibrated
        && config.turret_servo_test.pair_verified
        && configurationProblem() == null;
  }

  double rawVoltage() {
    return servos.leftVoltage();
  }

  @Nullable
  String blockedReason() {
    return blocked;
  }

  @Nullable
  String faultReason() {
    return fault;
  }

  void clearFault() {
    fault = null;
    manualReleaseRequired = true;
  }

  private static boolean validVoltage(double volts) {
    return TurretLaw.axonFeedbackValid(volts, config.turret.analog_encoder.full_scale_voltage);
  }

  /** One shaft's turret angle; NaN when its feedback is out of range. */
  private static double angle(boolean left, double volts) {
    var c = config.turret.dual_position;
    var s = config.turret_servo_test;
    return TurretLaw.axonAngle(
        volts,
        config.turret.analog_encoder.full_scale_voltage,
        left ? c.left_reference_volts : c.right_reference_volts,
        left ? c.left_volts_per_position : c.right_volts_per_position,
        left ? s.left_reversed : s.right_reversed,
        degreesPerPosition());
  }

  double currentAngle() {
    return TurretLaw.axonMeanAngle(
        active && calibrated(),
        angle(true, servos.leftVoltage()),
        angle(false, servos.rightVoltage()));
  }

  boolean reachable(double degrees) {
    return TurretLaw.axonReachable(
        calibrated(),
        degrees,
        degreesPerPosition(),
        config.turret_servo_test.max_jog_offset,
        config.turret.travel.min_angle,
        config.turret.travel.max_angle);
  }

  boolean aimed(double desired, double tolerance) {
    // Both shafts must agree; averaging opposite errors must never open the feed gate.
    return TurretLaw.axonAimed(
        desired,
        angle(true, servos.leftVoltage()),
        angle(false, servos.rightVoltage()),
        tolerance,
        active,
        fault != null,
        calibrated(),
        degreesPerPosition(),
        config.turret_servo_test.max_jog_offset,
        config.turret.travel.min_angle,
        config.turret.travel.max_angle);
  }

  void periodic(boolean enabled, Turret.AimMode mode, double desired, double manualPower) {
    long now = clock.getAsLong();
    double dt = lastUpdate == Long.MIN_VALUE ? 0 : (now - lastUpdate) / 1e9;
    lastUpdate = now;
    try {
      var c = config.turret.dual_position;
      var s = config.turret_servo_test;
      String problem = configurationProblem();
      double left = servos.leftVoltage();
      double right = servos.rightVoltage();
      double enabledSec = (now - enabledAt) / 1e9;
      if (!Double.isFinite(left) || !Double.isFinite(right)) fault = "Invalid Axon feedback";
      if (TurretLaw.axonFeedbackMissing(
          active, enabledSec, c.feedback_startup_sec, validVoltage(left), validVoltage(right)))
        fault = "Axon feedback missing after PWM enable";
      boolean manual = mode == Turret.AimMode.MANUAL;
      if (!manual || manualPower == 0 || !enabled) manualReleaseRequired = false;
      int state =
          TurretLaw.axonState(
              enabled,
              mode == Turret.AimMode.IDLE,
              problem == null,
              fault != null,
              manual,
              manualPower,
              manualReleaseRequired,
              TurretLaw.axonManualTimedOut(
                  active, dt, c.max_update_gap_sec, enabledSec, s.max_enable_sec),
              calibrated(),
              desired);
      switch (state) {
        case TurretLaw.AXON_OFF -> blocked = "PWM disabled";
        case TurretLaw.AXON_BLOCKED -> blocked = fault != null ? fault : problem;
        case TurretLaw.AXON_MANUAL_INVALID -> {
          manualReleaseRequired = true;
          blocked = "Invalid manual request; return input to neutral";
        }
        case TurretLaw.AXON_MANUAL_NEUTRAL -> blocked = "Manual input neutral: PWM disabled";
        case TurretLaw.AXON_MANUAL_REARM -> blocked = "Return manual input to neutral to re-arm";
        case TurretLaw.AXON_MANUAL_TIMEOUT -> {
          manualReleaseRequired = true;
          blocked = "Manual timeout: return input to neutral";
        }
        case TurretLaw.AXON_MANUAL_DRIVE -> {
          if (!active) {
            enabledAt = now;
            dt = 0;
          }
          offset =
              TurretLaw.axonManualOffset(
                  offset, manualPower, c.manual_position_per_sec, dt, s.max_jog_offset);
          blocked = null;
        }
        case TurretLaw.AXON_AIM_LOCKED ->
            blocked = "Automatic aim locked: verify pair, forward zero, sweep and CCW direction";
        case TurretLaw.AXON_AIM_INVALID -> blocked = "Invalid aim input";
        default -> {
          if (!active) enabledAt = now;
          offset =
              TurretLaw.axonAimOffset(
                  desired,
                  config.turret.travel.min_angle,
                  config.turret.travel.max_angle,
                  degreesPerPosition(),
                  s.max_jog_offset);
          blocked = reachable(desired) ? null : "Target beyond verified positional travel";
        }
      }
      active = TurretLaw.axonDrives(state);
      servos.write(active, active, offset);
    } catch (RuntimeException e) {
      active = false;
      fault = "Axon hardware read/write failure";
      servos.write(false, false, 0);
      throw e;
    }
  }
}
