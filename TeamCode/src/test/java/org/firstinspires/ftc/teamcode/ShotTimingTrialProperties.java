package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import net.jqwik.api.Arbitraries;
import net.jqwik.api.Arbitrary;
import net.jqwik.api.Combinators;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.Provide;
import net.jqwik.api.Tuple;
import net.jqwik.api.constraints.IntRange;
import org.firstinspires.ftc.teamcode.utilities.FlywheelDipDetector;
import org.firstinspires.ftc.teamcode.utilities.ShotTimingTrial;

/**
 * {@link ShotTimingTrial} fed random flywheel traces. Whatever the wheel does, a trial never counts
 * more balls than it was loaded with, never counts one before the wheel first reached speed, keeps
 * its ball times in order, and only completes with every ball counted and the wheel recovered; a
 * finished or cancelled trial never changes again.
 */
class ShotTimingTrialProperties {

  record Sample(double velocity, boolean ready, long dtMs) {}

  @Property(tries = 1000)
  void aTrialNeverOvercountsAndOnlyCompletesWhenAllBallsAreOutAndTheWheelRecovered(
      @ForAll @IntRange(min = 1, max = 4) int expected,
      @ForAll("traces") List<Sample> trace,
      @ForAll @IntRange(min = -1, max = 80) int cancelAt) {
    FlywheelDipDetector detector = new FlywheelDipDetector(.09, .05, .03, 120);
    ShotTimingTrial trial = new ShotTimingTrial(detector, expected, 0);
    long now = 0;
    boolean everReady = false;
    for (int i = 0; i < trace.size(); i++) {
      if (i == cancelAt) trial.cancel();
      Sample sample = trace.get(i);
      ShotTimingTrial.State before = trial.state();
      int countBefore = trial.ballTimesMs().size();
      now += sample.dtMs();
      everReady |= sample.ready();

      boolean ball = trial.update(sample.velocity(), sample.ready(), now);
      List<Integer> times = trial.ballTimesMs();

      assertTrue(times.size() <= expected, "counted " + times.size() + " of " + expected);
      for (int k = 1; k < times.size(); k++) {
        assertTrue(times.get(k - 1) <= times.get(k), "ball times out of order: " + times);
      }
      assertEquals(countBefore + (ball ? 1 : 0), times.size());
      if (ball) {
        assertTrue(everReady, "counted a ball before the wheel ever reached speed");
      }
      if (before != ShotTimingTrial.State.RUNNING) {
        assertFalse(ball);
        assertEquals(before, trial.state(), "a finished trial changed");
      }
      if (trial.state() == ShotTimingTrial.State.COMPLETE) {
        assertEquals(expected, times.size());
        assertFalse(detector.isInDip(), "completed while the wheel was still in a dip");
      }
    }
  }

  @Provide
  Arbitrary<List<Sample>> traces() {
    // Mostly near speed with deep dips, the shape a real volley has, plus arbitrary noise.
    Arbitrary<Double> velocity =
        Arbitraries.frequencyOf(
            Tuple.of(5, Arbitraries.doubles().between(900, 1100)),
            Tuple.of(3, Arbitraries.doubles().between(500, 900)),
            Tuple.of(1, Arbitraries.doubles().between(0, 500)));
    return Combinators.combine(
            velocity,
            Arbitraries.frequency(Tuple.of(2, true), Tuple.of(1, false)),
            Arbitraries.longs().between(1, 200))
        .as(Sample::new)
        .list()
        .ofMinSize(1)
        .ofMaxSize(80);
  }
}
