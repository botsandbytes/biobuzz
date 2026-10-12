package org.firstinspires.ftc.teamcode.utilities;

/** Conservative room reserved for ramp-down, measured braking, and loop reaction time. */
public final class SlewTravelBudget {
  private SlewTravelBudget() {}

  public static double stoppingRoom(
      double speed,
      double power,
      double rampTime,
      double linear,
      double quadratic,
      double safetyFactor,
      double reactionTime,
      double margin) {
    if (!Double.isFinite(speed)
        || !Double.isFinite(power)
        || !Double.isFinite(rampTime)
        || !Double.isFinite(linear)
        || !Double.isFinite(quadratic)
        || !Double.isFinite(safetyFactor)
        || safetyFactor <= 0
        || safetyFactor > 1
        || !Double.isFinite(reactionTime)
        || !Double.isFinite(margin)
        || rampTime < 0
        || linear < 0
        || quadratic < 0
        || reactionTime < 0
        || margin < 0) return Double.POSITIVE_INFINITY;
    double velocity = Math.abs(speed);
    // Reserve the FULL current-speed ramp distance, not an assumption of linear deceleration.
    return velocity * (Math.abs(power) * rampTime + reactionTime)
        + Math.abs(Casablanca.brakingDistance(velocity, linear, quadratic)) / safetyFactor
        + margin;
  }
}
