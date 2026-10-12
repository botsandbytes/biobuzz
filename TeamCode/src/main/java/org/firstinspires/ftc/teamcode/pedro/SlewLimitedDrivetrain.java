package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import java.util.Map;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.records.SlewProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.DriveSlewLimiter;

/** Applies Casablanca's ramp to FOLLOW/HOLD output without changing Pedro's implementation. */
public final class SlewLimitedDrivetrain implements Drivetrain {
  private final Drivetrain delegate;
  private final DriveSlewLimiter limiter = new DriveSlewLimiter();
  private final LongSupplier clock;
  private long previousTime;

  public SlewLimitedDrivetrain(Drivetrain delegate) {
    this(delegate, System::nanoTime);
  }

  public SlewLimitedDrivetrain(Drivetrain delegate, LongSupplier clock) {
    this.delegate = delegate;
    this.clock = clock;
    previousTime = clock.getAsLong();
  }

  @Override
  public void drive(DrivePowers powers, boolean manual) {
    long now = clock.getAsLong();
    double dt = (now - previousTime) * 1e-9;
    previousTime = now;
    if (!DriveSlewLimiter.finite(powers)) {
      stop(true);
      return;
    }
    var s = config.casablanca.smoothing;
    // Manual commands were already smoothed BEFORE Casablanca's collision protection.
    // Never delay a safety correction by filtering the manual output a second time.
    if (manual || !s.auto_enabled) {
      limiter.reset(powers);
      delegate.drive(powers, manual);
      return;
    }
    delegate.drive(
        limiter.calculate(powers, Math.min(dt, s.max_loop_dt), SlewProfile.configured()), false);
  }

  @Override
  public void stop() {
    limiter.reset();
    previousTime = clock.getAsLong();
    delegate.stop();
  }

  @Override
  public void stop(boolean brake) {
    limiter.reset();
    previousTime = clock.getAsLong();
    delegate.stop(brake);
  }

  @Override
  public double maxScaling(DrivePowers current, DrivePowers delta) {
    return delegate.maxScaling(current, delta);
  }

  @Override
  public double interpolateVelocity(double xRadius, double yRadius, double theta) {
    return delegate.interpolateVelocity(xRadius, yRadius, theta);
  }

  @Override
  public double interpolateAcceleration(double xRadius, double yRadius, double theta) {
    return delegate.interpolateAcceleration(xRadius, yRadius, theta);
  }

  @Override
  public Map<String, Object> debug() {
    return delegate.debug();
  }
}
