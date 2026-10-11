package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.records.SlewProfile;
import org.firstinspires.ftc.teamcode.records.SlewTuning;

/** Projects requested power onto permitted signed rate intervals without cross-axis delay. */
public final class DriveSlewLimiter {
  private DrivePowers output = DrivePowers.zero();

  public DrivePowers calculate(DrivePowers target, double dt, SlewProfile profile) {
    if (!finite(target) || !Double.isFinite(dt) || dt < 0 || profile == null) {
      reset();
      return output;
    }
    output =
        new DrivePowers(
            axis(output.forward(), target.forward(), dt, profile.forward()),
            axis(output.strafe(), target.strafe(), dt, profile.strafe()),
            axis(output.turn(), target.turn(), dt, profile.turn()));
    return output;
  }

  private static double axis(double current, double target, double dt, SlewTuning limits) {
    double time = target < current ? limits.decreaseTime() : limits.increaseTime();
    return time == 0 ? target : limit(current, target, dt / time);
  }

  public DrivePowers output() {
    return output;
  }

  public void reset(DrivePowers powers) {
    output = finite(powers) ? powers : DrivePowers.zero();
  }

  public void reset() {
    reset(DrivePowers.zero());
  }

  public static double limit(double current, double target, double maxChange) {
    return current + Math.clamp(target - current, -maxChange, maxChange);
  }

  public static boolean finite(DrivePowers powers) {
    return powers != null
        && Double.isFinite(powers.forward())
        && Double.isFinite(powers.strafe())
        && Double.isFinite(powers.turn());
  }
}
