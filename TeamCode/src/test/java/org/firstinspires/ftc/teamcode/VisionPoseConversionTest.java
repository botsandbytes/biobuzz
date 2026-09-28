package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.utilities.VisionUtil;
import org.junit.Test;

public class VisionPoseConversionTest {
  @Test
  public void invertedFtcCoordinatesPreservePriorFrameMapping() {
    Pose pose = VisionUtil.fromInvertedFtcRobotPose(12, -34, Math.PI / 3);
    assertEquals(-34, pose.x(), 1e-9);
    assertEquals(-12, pose.y(), 1e-9);
    assertEquals(Math.PI / 3, pose.heading(), 1e-9);

    Pose opposite = VisionUtil.fromInvertedFtcRobotPose(-9, 20, 0);
    assertEquals(20, opposite.x(), 1e-9);
    assertEquals(9, opposite.y(), 1e-9);
  }
}
