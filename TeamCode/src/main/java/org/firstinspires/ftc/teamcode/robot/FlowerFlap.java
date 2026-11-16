package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Flower flap: the drivers move it in TeleOp; AUTO raises it to drive and lowers it only at a
 * FLOWER. Commands select targets; only Robot.update() calls the hardware writer.
 */
public final class FlowerFlap {
  public enum Target {
    NONE,
    UP,
    DOWN
  }

  private final @Nullable ServoImplEx servo;
  private Target target = Target.NONE;
  private Target lastEndpoint = Target.UP;

  /** Calibration OpModes ({@code mapped} false) neither map nor write the flap hardware. */
  public FlowerFlap(HardwareMap hardwareMap, boolean mapped) {
    servo =
        mapped && config.flap.enabled
            ? hardwareMap.get(ServoImplEx.class, config.flap.servo_name)
            : null;
    if (servo != null && configurationProblem() == null) {
      // Read the retained command without enabling PWM. The SDK can reset the PWM range at INIT,
      // so compare pulse widths rather than comparing normalized positions from different ranges.
      double position = servo.getPosition();
      var range = servo.getPwmRange();
      if (Double.isFinite(position) && range != null) {
        if (servo.getDirection() == Servo.Direction.REVERSE) position = 1 - position;
        double pulse = Range.scale(position, 0, 1, range.usPulseLower, range.usPulseUpper);
        var c = config.flap;
        double configuredPosition = Range.scale(pulse, c.pwm_lower_us, c.pwm_upper_us, 0, 1);
        if (c.reversed) configuredPosition = 1 - configuredPosition;
        double angle = configuredPosition * c.servo_range_deg;
        if (Math.abs(angle - c.down_angle_deg) < Math.abs(angle - c.up_angle_deg)) {
          lastEndpoint = Target.DOWN;
        }
      }
    }
  }

  public void toggle() {
    setTarget(lastEndpoint == Target.DOWN ? Target.UP : Target.DOWN);
  }

  public void stop() {
    target = Target.NONE;
  }

  public void setTarget(Target target) {
    this.target = target;
    if (target != Target.NONE) lastEndpoint = target;
  }

  /** SDK command readback, not a measurement of the flap's physical angle. */
  public String outputStatus() {
    if (servo == null) return "Not mapped";
    var range = servo.getPwmRange();
    return String.format(
        Locale.ROOT,
        "SDK position %.4f; PWM enabled %s; range %.0f..%.0f us; %s; no angle sensor",
        servo.getPosition(),
        servo.isPwmEnabled(),
        range.usPulseLower,
        range.usPulseUpper,
        servo.getDirection());
  }

  public Target target() {
    return target;
  }

  /** Configured endpoint limits and SDK command readback, in normalized servo units. */
  public String positionStatus() {
    String problem = configurationProblem();
    if (problem != null) return problem;
    var c = config.flap;
    double up = c.up_angle_deg / c.servo_range_deg;
    double down = c.down_angle_deg / c.servo_range_deg;
    double current = servo == null ? Double.NaN : servo.getPosition();
    return String.format(
        Locale.ROOT,
        "min %.3f; max %.3f; current %s (SDK command, not measured)",
        Math.min(up, down),
        Math.max(up, down),
        Double.isFinite(current) ? String.format(Locale.ROOT, "%.3f", current) : "unavailable");
  }

  public Command toggleCommand() {
    return Command.build().setStart(this::toggle).setDone(() -> true).requiring(this);
  }

  public Command upCommand() {
    return Command.build().setStart(() -> setTarget(Target.UP)).setDone(() -> true).requiring(this);
  }

  public static @Nullable String configurationProblem() {
    var c = config.flap;
    if (c.servo_name.isBlank()) return "flap.servo_name must name the configured positional servo";
    if (!(c.servo_range_deg > 0
        && Double.isFinite(c.servo_range_deg)
        && c.up_angle_deg >= 0
        && c.up_angle_deg <= c.servo_range_deg
        && c.down_angle_deg >= 0
        && c.down_angle_deg <= c.servo_range_deg
        && c.up_angle_deg != c.down_angle_deg)) {
      return "flap: distinct up/down angles must lie within servo_range_deg";
    }
    if (!(c.pwm_lower_us >= 500 && c.pwm_upper_us <= 2500 && c.pwm_lower_us < c.pwm_upper_us)) {
      return "flap: invalid positional servo PWM range";
    }
    return null;
  }

  public String status() {
    if (servo == null || !config.flap.enabled) return "Disabled";
    String problem = configurationProblem();
    if (problem != null) return problem;
    String state = target == Target.NONE ? "Unpowered" : target + " target";
    return String.format(
        Locale.ROOT,
        "%s; next toggle %s (UP %.0f / DOWN %.0f commanded deg; no position sensor)",
        state,
        lastEndpoint == Target.DOWN ? Target.UP : Target.DOWN,
        config.flap.up_angle_deg,
        config.flap.down_angle_deg);
  }

  /** Sole actuator writer. No centering or movement until the operator selects a target. */
  void periodic() {
    if (servo == null) return;
    if (!config.flap.enabled || target == Target.NONE || configurationProblem() != null) {
      servo.setPwmDisable();
      return;
    }
    var c = config.flap;
    servo.setPwmRange(new PwmControl.PwmRange(c.pwm_lower_us, c.pwm_upper_us));
    servo.setDirection(c.reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    double angle = target == Target.DOWN ? c.down_angle_deg : c.up_angle_deg;
    servo.setPosition(angle / c.servo_range_deg);
    servo.setPwmEnable();
  }
}
