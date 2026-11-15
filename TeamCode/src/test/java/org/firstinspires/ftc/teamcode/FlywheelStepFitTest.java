package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.constraints.Scale;
import org.firstinspires.ftc.teamcode.ballistics.FlywheelStepFit;
import org.junit.jupiter.api.Test;

/** {@link FlywheelStepFit}: the least-squares first-order-plus-dead-time fit. */
class FlywheelStepFitTest {

  /** A first-order wheel with dead time, sampled every {@code dt}, with uniform noise. */
  private static double[][] step(
      double v0,
      double vFinal,
      double tau,
      double lag,
      double dt,
      double durationSec,
      double noise,
      long seed) {
    Random random = new Random(seed);
    int n = (int) (durationSec / dt);
    double[] t = new double[n];
    double[] v = new double[n];
    for (int i = 0; i < n; i++) {
      t[i] = (i + 1) * dt;
      double moved = t[i] <= lag ? 0.0 : 1.0 - Math.exp(-(t[i] - lag) / tau);
      v[i] = v0 + (vFinal - v0) * moved + noise * (2 * random.nextDouble() - 1);
    }
    return new double[][] {t, v};
  }

  @Test
  void recoversTauAndLagFromAFineCleanStep() {
    double[][] s = step(400, 1100, 0.5, 0.04, 0.001, 4.0, 0.0, 1);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 400, 1100);
    assertTrue(fit.valid());
    assertEquals(0.5, fit.tauSec(), 0.005);
    assertEquals(0.04, fit.lagSec(), 0.003);
  }

  @Test
  void aFallingStepFitsTheSame() {
    double[][] s = step(1100, 400, 0.3, 0.02, 0.001, 3.0, 0.0, 1);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 1100, 400);
    assertTrue(fit.valid());
    assertEquals(0.3, fit.tauSec(), 0.005);
    assertEquals(0.02, fit.lagSec(), 0.003);
  }

  @Test
  void refusesTooSmallAStep() {
    double[][] s = step(800, 820, 0.5, 0.0, 0.01, 3.0, 0.0, 1);
    assertFalse(FlywheelStepFit.fit(s[0], s[1], 800, 820).valid());
    assertFalse(FlywheelStepFit.fit(s[0], s[1], 800, Double.NaN).valid());
  }

  @Test
  void refusesAStepCutShort() {
    // 0.25 s of a 1 s wheel reaches 22% of the step: under five samples in the window.
    double[][] s = step(400, 1100, 1.0, 0.0, 0.01, 0.25, 0.0, 1);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 400, 1100);
    assertFalse(fit.valid());
    assertTrue(Double.isNaN(fit.tauSec()));
  }

  @Test
  void refusesMismatchedArrays() {
    assertFalse(FlywheelStepFit.fit(new double[3], new double[2], 0, 500).valid());
  }

  @Test
  void aSingleSpikeBarelyMovesTheFit() {
    double[][] s = step(400, 1100, 0.5, 0.0, 0.01, 4.0, 0.0, 1);
    FlywheelStepFit.Result clean = FlywheelStepFit.fit(s[0], s[1], 400, 1100);
    s[1][2] = 1100; // a full-step spike at 30 ms: not a confirmed crossing, one sample of ~150
    FlywheelStepFit.Result spiked = FlywheelStepFit.fit(s[0], s[1], 400, 1100);
    assertEquals(clean.tauSec(), spiked.tauSec(), 0.02 * clean.tauSec());
    assertEquals(clean.lagSec(), spiked.lagSec(), 0.005);
  }

  @Test
  void aZeroLagNeverReportsNegative() {
    double[][] s = step(400, 1100, 0.5, 0.0, 0.02, 4.0, 0.0, 1);
    assertTrue(FlywheelStepFit.fit(s[0], s[1], 400, 1100).lagSec() >= 0.0);
  }

  /**
   * At the characterization step test's loop (a few ms: it only reads the hub) with realistic
   * encoder noise (up to 15 ticks/s on a step of 500+), the fit's standard errors are honest (the
   * true values lie within 6 of them; 4.4 is the worst in 70,000 simulated fits), and wherever it
   * reports tau within 3% or the lag within 7.5 ms, it is: 7.5 ms is kick_lag_margin_sec, the error
   * FlywheelClosedLoopProperties gives the kick. Not every wheel in this range can be pinned that
   * well from one step (a slow wheel sampled coarsely through heavy noise misses 7.5 ms about one
   * fit in thirty); then the fit says so, and the OpMode warns rather than reporting the lag.
   */
  @Property(tries = 300)
  void fitsAnyPlausibleWheelAtTheOpModesLoopRate(
      @ForAll @DoubleRange(min = 0.15, max = 1.0) double tau,
      @ForAll @DoubleRange(min = 0.0, max = 0.06) double lag,
      @ForAll @DoubleRange(min = 0.002, max = 0.010) @Scale(4) double dt,
      @ForAll @DoubleRange(min = 500, max = 900) double rise,
      @ForAll @DoubleRange(min = 0, max = 15) double noise,
      @ForAll long seed) {
    double[][] s = step(400, 400 + rise, tau, lag, dt, 5 * tau + 0.5, noise, seed);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 400, 400 + rise);
    assertTrue(fit.valid(), fit.problem());
    assertTrue(Double.isFinite(fit.tauSigmaSec()) && Double.isFinite(fit.lagSigmaSec()), "sigma");
    // Floors for a noise-free step, where the standard errors are ~0 and the solver's tolerance is
    // what remains.
    assertEquals(tau, fit.tauSec(), 6 * fit.tauSigmaSec() + 1e-3 * tau, "tau vs its error");
    assertEquals(lag, fit.lagSec(), 6 * fit.lagSigmaSec() + 5e-4, "lag vs its error");
    if (fit.tauWithin(0.03)) {
      assertEquals(tau, fit.tauSec(), 0.03 * tau, "tau");
    }
    if (fit.lagWithin(0.0075)) {
      assertEquals(lag, fit.lagSec(), 0.0075, "lag");
    }
  }

  /**
   * The sample that failed the old fixed 7.5 ms bound: a slow wheel through heavy noise, fitted 7.5
   * ms short. The fit is right about how unsure it is (2.25 ms standard error), so it does not
   * claim the kick's margin.
   */
  @Test
  void anImpreciseLagSaysSoInsteadOfClaimingTheKicksMargin() {
    double[][] s = step(400, 900, 0.74, 0.05, 0.0062, 5 * 0.74 + 0.5, 13.98, -124);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 400, 900);
    assertTrue(fit.valid());
    assertTrue(Math.abs(fit.lagSec() - 0.05) > 0.0074, "the recorded miss");
    assertEquals(0.05, fit.lagSec(), 6 * fit.lagSigmaSec());
    assertFalse(fit.lagWithin(0.0075));
  }

  /** This robot's wheel (wheel_tau_sec 0.276) at a 4 ms loop with 10 ticks/s of noise. */
  @Test
  void theRobotsWheelIsPinnedWellEnoughForTheKick() {
    double[][] s = step(400, 900, 0.276, 0.045, 0.004, 4.5, 10, 1);
    FlywheelStepFit.Result fit = FlywheelStepFit.fit(s[0], s[1], 400, 900);
    assertTrue(fit.lagWithin(0.0075), "lag standard error " + fit.lagSigmaSec());
    assertTrue(fit.tauWithin(0.03), "tau standard error " + fit.tauSigmaSec());
    assertEquals(0.045, fit.lagSec(), 0.0075);
  }
}
