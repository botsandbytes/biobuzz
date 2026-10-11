package org.firstinspires.ftc.teamcode.records;

/** Seconds per unit power change in each signed direction. Zero means unrestricted. */
public record SlewTuning(double increaseTime, double decreaseTime) {
  public SlewTuning {
    if (!Double.isFinite(increaseTime)
        || increaseTime < 0
        || !Double.isFinite(decreaseTime)
        || decreaseTime < 0)
      throw new IllegalArgumentException("Slew times must be finite and nonnegative");
  }

  public double longestTime() {
    return Math.max(increaseTime, decreaseTime);
  }
}
