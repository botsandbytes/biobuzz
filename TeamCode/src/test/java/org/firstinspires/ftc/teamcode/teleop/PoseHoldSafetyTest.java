package org.firstinspires.ftc.teamcode.teleop;

import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class PoseHoldSafetyTest {
  /** These tests check the refusal itself; FreshConfigExtension restores the shipped value. */
  @BeforeEach
  public void enableFootprintSafety() {
    config.sentinel.footprint_safety_enabled = true;
  }

  @Test
  public void inPlaceHoldRejectsIntermediateHiveAndWallCollisions() {
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      var redFrame = new FieldModel(Alliance.RED).hiveFrame();
      var chassis = config.chassis;
      double half = chassis.robot_width_in / 2.0;
      double front =
          chassis.robot_length_in
              - (chassis.back_axle_from_back_in + chassis.front_axle_from_back_in) / 2.0;
      double peak = Math.atan2(half, front);
      double sweep = Math.toRadians(10);
      double radius = Math.hypot(half, front);
      double endpoint = radius * Math.cos(sweep);
      double distance = (endpoint + radius) / 2;
      for (boolean hive : new boolean[] {true, false}) {
        TeleOp op = new TeleOp();
        op.follower = mock(Follower.class);
        op.sentinel = new Sentinel(field);
        double x =
            hive
                ? redFrame.getMinX() - distance
                : distance + config.sentinel.route_wall_clearance_in;
        double y = hive ? redFrame.centre().y : 25;
        double heading = hive ? peak : peak + Math.PI;
        Pose from = field.fromRedFrame(new Pose(x, y, heading - sweep));
        Pose target = field.fromRedFrame(new Pose(x, y, heading + sweep));
        when(op.follower.pose()).thenReturn(from);
        op.holdPoseIfClear(target);
        verify(op.follower).stop();
        verify(op.follower, never()).hold(any());
      }
    }
  }

  @Test
  public void holdsRefuseHiveCrossingsAndAllowClearApproachesForBothAlliances() {
    for (Alliance alliance : Alliance.values()) {
      TeleOp op = new TeleOp();
      op.follower = mock(Follower.class);
      op.sentinel = new Sentinel(alliance);
      FieldModel field = new FieldModel(alliance);
      Pose from = field.fromRedFrame(new Pose(58, 27, Math.PI / 2));
      Pose blocked = field.fromRedFrame(new Pose(58, 114, Math.PI / 2));
      Pose clear = field.fromRedFrame(new Pose(30, 27, Math.PI / 2));
      when(op.follower.pose()).thenReturn(from);
      op.holdPoseIfClear(blocked);
      verify(op.follower).stop();
      verify(op.follower, never()).hold(blocked);
      op.holdPoseIfClear(clear);
      verify(op.follower).hold(clear);
      op.holdPoseIfClear(from);
      verify(op.follower).hold(from);
    }
  }
}
