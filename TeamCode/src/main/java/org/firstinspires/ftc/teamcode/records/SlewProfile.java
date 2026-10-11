package org.firstinspires.ftc.teamcode.records;

import java.util.Objects;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

public record SlewProfile(SlewTuning forward, SlewTuning strafe, SlewTuning turn) {
  public enum Axis {
    FORWARD,
    STRAFE,
    TURN
  }

  public SlewProfile {
    Objects.requireNonNull(forward);
    Objects.requireNonNull(strafe);
    Objects.requireNonNull(turn);
  }

  public static SlewProfile configured() {
    var s = config.casablanca.smoothing;
    return new SlewProfile(
        new SlewTuning(s.forward.increase_time, s.forward.decrease_time),
        new SlewTuning(s.strafe.increase_time, s.strafe.decrease_time),
        new SlewTuning(s.turn.increase_time, s.turn.decrease_time));
  }

  public SlewTuning axis(Axis axis) {
    return switch (axis) {
      case FORWARD -> forward;
      case STRAFE -> strafe;
      case TURN -> turn;
    };
  }

  public SlewProfile with(Axis axis, SlewTuning value) {
    return switch (axis) {
      case FORWARD -> new SlewProfile(value, strafe, turn);
      case STRAFE -> new SlewProfile(forward, value, turn);
      case TURN -> new SlewProfile(forward, strafe, value);
    };
  }
}
