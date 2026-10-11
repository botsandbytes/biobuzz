package org.firstinspires.ftc.teamcode.utilities;

/** Scores a measured closed-loop heading step. No controller or relay is implemented here. */
public final class HeadingStepResponse {
  public enum Status {
    RUNNING,
    PASS,
    FAIL,
    INVALID
  }

  private final double sign, tolerance, rateTolerance, overshootLimit, settleTime, duration;
  private double elapsed, settled, firstSettled = Double.NaN, overshoot;
  private Status status = Status.RUNNING;

  public HeadingStepResponse(
      double step,
      double tolerance,
      double rateTolerance,
      double overshootLimit,
      double settleTime,
      double duration) {
    if (!Double.isFinite(step)
        || step == 0
        || !positive(tolerance)
        || !positive(rateTolerance)
        || !positive(overshootLimit)
        || !positive(settleTime)
        || !positive(duration)
        || duration <= settleTime
        || Math.abs(step) <= tolerance)
      throw new IllegalArgumentException("Invalid heading step limits");
    sign = Math.signum(step);
    this.tolerance = tolerance;
    this.rateTolerance = rateTolerance;
    this.overshootLimit = overshootLimit;
    this.settleTime = settleTime;
    this.duration = duration;
  }

  public Status update(double error, double angularRate, double dt) {
    if (status != Status.RUNNING) return status;
    if (!Double.isFinite(error) || !Double.isFinite(angularRate) || !positive(dt)) {
      status = Status.INVALID;
      return status;
    }
    elapsed += dt;
    overshoot = Math.max(overshoot, -sign * error);
    if (overshoot > overshootLimit) {
      status = Status.FAIL;
      return status;
    }
    boolean inBand = Math.abs(error) <= tolerance && Math.abs(angularRate) <= rateTolerance;
    settled = inBand ? settled + dt : 0;
    // A later departure invalidates an earlier settling time.
    if (!inBand) firstSettled = Double.NaN;
    if (settled >= settleTime && Double.isNaN(firstSettled)) firstSettled = elapsed;
    if (elapsed >= duration) status = settled >= settleTime ? Status.PASS : Status.FAIL;
    return status;
  }

  public double settlingSeconds() {
    return status == Status.PASS ? firstSettled : Double.POSITIVE_INFINITY;
  }

  public double overshoot() {
    return overshoot;
  }

  public static double gainAt(double min, double max, int count, int index) {
    if (!Double.isFinite(min)
        || min < 0
        || !Double.isFinite(max)
        || max < min
        || count < 1
        || index < 0
        || index >= count) throw new IllegalArgumentException("Invalid gain grid");
    return count == 1 ? min : Math.fma((double) index / (count - 1), max - min, min);
  }

  private static boolean positive(double x) {
    return Double.isFinite(x) && x > 0;
  }
}
