package org.firstinspires.ftc.teamcode.records;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Pitch/roll sample in degrees, timestamped in the System.nanoTime clock domain. */
public record TiltReading(double pitch, double roll, long nanos) {
  public boolean finite() {
    return Double.isFinite(pitch) && Double.isFinite(roll);
  }

  public double secondsSince(TiltReading previous) {
    return (nanos - previous.nanos) * 1e-9;
  }

  public double pitchRate(TiltReading previous) {
    return secondsSince(previous) > 0
        ? AngleUnit.normalizeDegrees(pitch - previous.pitch) / secondsSince(previous)
        : Double.NaN;
  }

  public double rollRate(TiltReading previous) {
    return secondsSince(previous) > 0
        ? AngleUnit.normalizeDegrees(roll - previous.roll) / secondsSince(previous)
        : Double.NaN;
  }
}
