package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.util.ArrayList;
import java.util.List;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.Tuple;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.lifecycle.AfterProperty;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * The positional Axon turret (turret.dual_position) run as the real {@link Turret} and {@link
 * DualPositionTurret} against mocked servos and feedback. Its decisions are {@link TurretLaw}'s
 * {@code axon*} methods, proven one call at a time; these check what they add up to over many
 * loops: no servo is ever commanded outside its center +/- max_jog_offset, PWM is never on while
 * the turret is disabled, IDLE, or (outside a manual jog) uncalibrated, and the aim claim that
 * opens the feed gate needs both shafts within tolerance of a reachable target. The CR-plant
 * properties ({@code TurretClosedLoopProperties}) do not cover this backend.
 */
class DualPositionTurretProperties {
  private static final Pose POSE = new Pose(0, 0, 0);

  private final boolean savedDual = config.turret.dual_position.enabled;
  private final boolean savedCalibrated = config.turret.dual_position.angle_calibrated;
  private final boolean savedPair = config.turret_servo_test.pair_verified;
  private final double savedMinDeg = config.turret.tolerance.min_deg;
  private final double savedMaxDeg = config.turret.tolerance.max_deg;
  private final double savedNearVal = config.turret.tolerance.near_val;

  @AfterProperty
  void restore() {
    config.turret.dual_position.enabled = savedDual;
    config.turret.dual_position.angle_calibrated = savedCalibrated;
    config.turret_servo_test.pair_verified = savedPair;
    config.turret.tolerance.min_deg = savedMinDeg;
    config.turret.tolerance.max_deg = savedMaxDeg;
    config.turret.tolerance.near_val = savedNearVal;
  }

  /** Mocked hardware that records every position and PWM state the turret writes. */
  private static final class Rig {
    final ServoImplEx left = mock(ServoImplEx.class);
    final ServoImplEx right = mock(ServoImplEx.class);
    final AnalogInput leftAnalog = mock(AnalogInput.class);
    final AnalogInput rightAnalog = mock(AnalogInput.class);
    final List<Double> leftPositions = new ArrayList<>();
    final List<Double> rightPositions = new ArrayList<>();
    final boolean[] pwm = new boolean[2];
    long now;
    final Turret turret;

    Rig() {
      config.turret.dual_position.enabled = true;
      var s = config.turret_servo_test;
      HardwareMap hw = mock(HardwareMap.class);
      when(hw.get(ServoImplEx.class, s.left_servo_name)).thenReturn(left);
      when(hw.get(ServoImplEx.class, s.right_servo_name)).thenReturn(right);
      when(hw.get(AnalogInput.class, s.left_analog_name)).thenReturn(leftAnalog);
      when(hw.get(AnalogInput.class, s.right_analog_name)).thenReturn(rightAnalog);
      record(left, leftPositions, 0);
      record(right, rightPositions, 1);
      shaftsAt(0, 0);
      turret = new Turret(hw, null, () -> POSE, Velocity::zero, () -> now);
    }

    private void record(ServoImplEx servo, List<Double> positions, int side) {
      doAnswer(
              inv -> {
                positions.add(inv.<Double>getArgument(0));
                return null;
              })
          .when(servo)
          .setPosition(anyDouble());
      doAnswer(
              inv -> {
                pwm[side] = true;
                return null;
              })
          .when(servo)
          .setPwmEnable();
      doAnswer(
              inv -> {
                pwm[side] = false;
                return null;
              })
          .when(servo)
          .setPwmDisable();
    }

    /** Feedback voltages for these shaft angles: the inverse of the configured mapping. */
    void shaftsAt(double leftDeg, double rightDeg) {
      var c = config.turret.dual_position;
      var s = config.turret_servo_test;
      double dpp = DualPositionTurret.degreesPerPosition();
      when(leftAnalog.getVoltage())
          .thenReturn(
              c.left_reference_volts
                  + leftDeg / dpp * (s.left_reversed ? -1 : 1) * c.left_volts_per_position);
      when(rightAnalog.getVoltage())
          .thenReturn(
              c.right_reference_volts
                  + rightDeg / dpp * (s.right_reversed ? -1 : 1) * c.right_volts_per_position);
    }
  }

  /** One loop's inputs. {@code leftLost} reads the left feedback as 0 V (unplugged). */
  record Step(
      boolean enabled,
      Turret.AimMode mode,
      double target,
      double manualPower,
      double leftDeg,
      double rightDeg,
      boolean leftLost,
      long dtMs) {}

  @Property(tries = 300)
  void servosStayInTheirWindowAndPwmIsOffWhenItMustBe(
      @ForAll boolean calibrated, @ForAll("steps") List<Step> steps) {
    config.turret.dual_position.angle_calibrated = calibrated;
    config.turret_servo_test.pair_verified = calibrated;
    var s = config.turret_servo_test;
    Rig rig = new Rig();
    for (Step step : steps) {
      rig.now += step.dtMs() * 1_000_000L;
      rig.shaftsAt(step.leftDeg(), step.rightDeg());
      if (step.leftLost()) when(rig.leftAnalog.getVoltage()).thenReturn(0.0);
      rig.turret.setEnabled(step.enabled());
      rig.turret.setHoldAngle(step.target());
      rig.turret.setManualPower(step.manualPower());
      rig.turret.setAimMode(step.mode());
      rig.turret.periodic();

      for (double p : rig.leftPositions) {
        assertTrue(Math.abs(p - s.left_center_position) <= s.max_jog_offset + 1e-12, "left " + p);
      }
      for (double p : rig.rightPositions) {
        assertTrue(Math.abs(p - s.right_center_position) <= s.max_jog_offset + 1e-12, "right " + p);
      }
      boolean anyPwm = rig.pwm[0] || rig.pwm[1];
      if (!step.enabled() || step.mode() == Turret.AimMode.IDLE) {
        assertFalse(anyPwm, "PWM on while disabled or IDLE: " + step);
      }
      if (!calibrated && step.mode() != Turret.AimMode.MANUAL) {
        assertFalse(anyPwm, "automatic aim powered an unverified pair: " + step);
      }
      if (rig.turret.isAimed(POSE)) {
        assertTrue(anyPwm && calibrated, "aim claimed without powered, verified servos: " + step);
      }
    }
  }

  @Property(tries = 1000)
  void theAimClaimNeedsBothShaftsWithinToleranceOfAReachableTarget(
      @ForAll @DoubleRange(min = -30, max = 30) double target,
      @ForAll @DoubleRange(min = -3, max = 3) double leftError,
      @ForAll @DoubleRange(min = -3, max = 3) double rightError,
      @ForAll @DoubleRange(min = 0.2, max = 2) double tolerance) {
    config.turret.dual_position.angle_calibrated = true;
    config.turret_servo_test.pair_verified = true;
    config.turret.tolerance.min_deg = tolerance;
    config.turret.tolerance.max_deg = tolerance;
    config.turret.tolerance.near_val = tolerance;
    Rig rig = new Rig();
    rig.turret.setTargetPoint(100, 0);
    rig.turret.setHoldAngle(target);
    rig.turret.setAimMode(Turret.AimMode.HOLD);
    rig.turret.periodic(); // powers the pair
    double held = rig.turret.getHoldAngle(); // the target, limited to the travel
    rig.shaftsAt(held + leftError, held + rightError);
    rig.turret.periodic();

    boolean reachable = !rig.turret.isTargetOutsideTravel();
    boolean aimed = rig.turret.isAimed(POSE);
    if (aimed) {
      assertTrue(reachable, "aimed at a target outside the verified window");
      assertTrue(Math.abs(leftError) < tolerance + 1e-9, "left shaft off by " + leftError);
      assertTrue(Math.abs(rightError) < tolerance + 1e-9, "right shaft off by " + rightError);
    }
    if (reachable
        && Math.abs(leftError) < 0.99 * tolerance
        && Math.abs(rightError) < 0.99 * tolerance) {
      assertTrue(aimed, "both shafts on target but no aim claim");
    }
  }

  @Provide
  Arbitrary<List<Step>> steps() {
    Arbitrary<Double> manual =
        Arbitraries.frequencyOf(
            Tuple.of(6, Arbitraries.doubles().between(-1.5, 1.5)),
            Tuple.of(2, Arbitraries.just(0.0)),
            Tuple.of(1, Arbitraries.just(Double.NaN)));
    Arbitrary<Double> shaft = Arbitraries.doubles().between(-40, 40);
    return Combinators.combine(
            Arbitraries.frequency(Tuple.of(3, true), Tuple.of(1, false)),
            Arbitraries.of(Turret.AimMode.class),
            Arbitraries.doubles().between(-60, 60),
            manual,
            shaft,
            shaft,
            Arbitraries.frequency(Tuple.of(1, true), Tuple.of(5, false)),
            Arbitraries.longs().between(0, 400))
        .as(Step::new)
        .list()
        .ofMinSize(1)
        .ofMaxSize(60);
  }
}
