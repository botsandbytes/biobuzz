package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Detects sustained chassis tilt/rate excursions; an IMU cannot prove wheel contact. */
public final class JumpDetector {
  public enum Status {
    CLEAR,
    JUMP,
    EMERGENCY,
    INVALID
  }

  private final double pitchZero, rollZero, tiltLimit, rateLimit, holdTime, emergencyTilt;
  private double excursionTime, peakTilt, peakRate;
  private boolean detected;

  public JumpDetector(
      double pitchZero,
      double rollZero,
      double tiltLimit,
      double rateLimit,
      double holdTime,
      double emergencyTilt) {
    if (!Double.isFinite(pitchZero)
        || !Double.isFinite(rollZero)
        || !positive(tiltLimit)
        || !positive(rateLimit)
        || !positive(holdTime)
        || !positive(emergencyTilt)
        || emergencyTilt <= tiltLimit) {
      throw new IllegalArgumentException("Invalid jump detection thresholds");
    }
    this.pitchZero = pitchZero;
    this.rollZero = rollZero;
    this.tiltLimit = tiltLimit;
    this.rateLimit = rateLimit;
    this.holdTime = holdTime;
    this.emergencyTilt = emergencyTilt;
  }

  public Status update(double pitch, double roll, double pitchRate, double rollRate, double dt) {
    if (!Double.isFinite(pitch)
        || !Double.isFinite(roll)
        || !Double.isFinite(pitchRate)
        || !Double.isFinite(rollRate)
        || !Double.isFinite(dt)
        || dt < 0) {
      return Status.INVALID;
    }
    double tilt =
        Math.hypot(
            AngleUnit.normalizeDegrees(pitch - pitchZero),
            AngleUnit.normalizeDegrees(roll - rollZero));
    double rate = Math.hypot(pitchRate, rollRate);
    peakTilt = Math.max(peakTilt, tilt);
    peakRate = Math.max(peakRate, rate);
    if (tilt >= emergencyTilt) return Status.EMERGENCY;
    excursionTime = tilt >= tiltLimit || rate >= rateLimit ? excursionTime + dt : 0;
    detected |= excursionTime >= holdTime;
    return detected ? Status.JUMP : Status.CLEAR;
  }

  public double peakTilt() {
    return peakTilt;
  }

  public double peakRate() {
    return peakRate;
  }

  public boolean detected() {
    return detected;
  }

  private static boolean positive(double value) {
    return Double.isFinite(value) && value > 0;
  }
}
