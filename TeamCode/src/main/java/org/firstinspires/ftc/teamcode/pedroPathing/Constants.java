package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/** Builds Pedro 3 from the robot's existing motor and Pinpoint setup. */
public final class Constants {
  private Constants() {}

  public static MecanumConfig drivetrainConfig() {
    return new MecanumConfig(
        c -> {
          c.frontLeftName.set("leftFront");
          c.backLeftName.set("leftBack");
          c.frontRightName.set("rightFront");
          c.backRightName.set("rightBack");
          c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
          c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
          c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
          c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        });
  }

  public static PinpointConfig localizerConfig() {
    return new PinpointConfig(
        c -> {
          c.name.set("pinpoint");
          c.xPodOffset.set(config.pedro.pinpoint_x_pod_offset);
          c.yPodOffset.set(config.pedro.pinpoint_y_pod_offset);
          c.offsetUnits.set(DistanceUnit.INCH);
          c.globalDistanceUnit.set(DistanceUnit.INCH);
          c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
          c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
          c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        });
  }

  public static ForesightConfig foresightConfig() {
    var p = config.pedro;
    return new ForesightConfig(
        c -> {
          c.headingFeedback.set(Controller.pid(p.heading_kp, 0, p.heading_kd));
          c.headingStaticFF.set(Controller.staticFeedforward(p.heading_ks));
          c.forwardTranslational.set(Controller.proportional(p.forward_kp));
          c.strafeTranslational.set(Controller.proportional(p.strafe_kp));
          c.coast.set(Controller.proportionalFeedforward(p.coast_kv));
          c.brake.set(Controller.proportionalFeedforward(p.brake_kv));
          c.linearBrakeCoefficients.set(Matrix.diag(p.forward_brake_linear, p.strafe_brake_linear));
          c.quadraticBrakeCoefficients.set(
              Matrix.diag(p.forward_brake_quadratic, p.strafe_brake_quadratic));
          c.headingBrakeCoefficients.set(
              Vector2D.cartesian(p.heading_brake_linear, p.heading_brake_quadratic));
          c.maxAchievableForwardVelocity.set(p.max_forward_velocity);
          c.maxAchievableStrafeVelocity.set(p.max_strafe_velocity);
          c.naturalForwardDeceleration.set(p.natural_forward_deceleration);
          c.naturalStrafeDeceleration.set(p.natural_strafe_deceleration);
          c.maxPathSpeed.set(p.max_path_speed);
        });
  }

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new Follower(
        new PinpointLocalizer(hardwareMap, localizerConfig()),
        new Mecanum(hardwareMap, drivetrainConfig()),
        new Foresight(foresightConfig()));
  }

  /** Pedro 3's Mecanum drivetrain already caches motor writes. */
  public static Follower createCachedFollower(HardwareMap hardwareMap) {
    return createFollower(hardwareMap);
  }
}
