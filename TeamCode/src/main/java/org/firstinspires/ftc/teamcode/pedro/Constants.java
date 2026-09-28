package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** QuickStart entry point backed by the team's configured Pedro 3 follower. */
public final class Constants {
  private Constants() {}

  public static Follower create(HardwareMap hardwareMap) {
    return org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
  }
}
