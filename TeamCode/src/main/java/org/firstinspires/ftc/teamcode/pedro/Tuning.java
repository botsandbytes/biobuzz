package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/** Robot-hosted Pedro 3 AutoTune procedures for this drivetrain and localizer. */
public final class Tuning {
  private Tuning() {}

  @Tuner
  public static Procedure mecanumTuner() {
    return new MecanumTuner();
  }

  @Tuner
  public static Procedure pinpointTuner() {
    return new PinpointTuner();
  }

  @Tuner
  public static Procedure foresightTuner() {
    return new ForesightTuner(
        h ->
            new PinpointLocalizer(
                h, org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConfig()),
        h ->
            new Mecanum(
                h, org.firstinspires.ftc.teamcode.pedroPathing.Constants.drivetrainConfig()));
  }

  @Tuner
  public static Procedure tests() {
    return new Tests(
        h ->
            new Mecanum(
                h, org.firstinspires.ftc.teamcode.pedroPathing.Constants.drivetrainConfig()),
        h ->
            new PinpointLocalizer(
                h, org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConfig()),
        () ->
            new Foresight(org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig()));
  }
}
