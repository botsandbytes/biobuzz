package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Test;

/** {@code sentinel.footprint_safety_enabled} switches the footprint refusals on and off. */
public class FootprintSafetySwitchTest {

  @Test
  public void withFootprintSafetyOffAHoldAcrossTheHiveIsDriven() {
    config.sentinel.footprint_safety_enabled = false;
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      TeleOp op = new TeleOp();
      op.follower = mock(Follower.class);
      op.sentinel = new Sentinel(field);
      // The same HIVE crossing PoseHoldSafetyTest refuses with the switch on.
      Pose from = field.fromRedFrame(new Pose(58, 27, Math.PI / 2));
      Pose across = field.fromRedFrame(new Pose(58, 114, Math.PI / 2));
      when(op.follower.pose()).thenReturn(from);
      op.holdPoseIfClear(across);
      verify(op.follower).hold(across);
      verify(op.follower, never()).stop();
    }
  }

  /**
   * A robot just clear of the HIVE frame turning hard enough that the predicted sweep reaches it:
   * the turn is refused only while footprint safety is on.
   */
  @Test
  public void theDriverTurnIsRefusedNearTheHiveOnlyWhileFootprintSafetyIsOn() {
    config.casablanca.smoothing.teleop_enabled = false;
    Sentinel sentinel = new Sentinel(Alliance.RED);
    double centre = config.field.size_in / 2.0;
    double protectedMaxX = sentinel.getProtectedZones().get(0).getMaxX();
    Pose beside = new Pose(protectedMaxX + config.sentinel.robot_width / 2.0 + 0.5, centre, 0);
    // One radian of predicted sweep; MathSafetyTest shows half a radian already reaches the frame.
    double omega = 1.0 / config.sentinel.rotation_lookahead_time;
    assertFalse(sentinel.isRotationSafe(beside, 1.0, 1.0), "the sweep reaches the HIVE frame");

    config.sentinel.footprint_safety_enabled = true;
    Casablanca on = new Casablanca(sentinel);
    on.adjustDriveInput(beside, Vector2D.zero(), omega, 0.0, 0.0, 1.0);
    assertFalse(on.getLastRotationSafe(), "footprint safety on: the turn is refused");

    config.sentinel.footprint_safety_enabled = false;
    Casablanca off = new Casablanca(sentinel);
    double turn = off.adjustDriveInput(beside, Vector2D.zero(), omega, 0.0, 0.0, 1.0)[2];
    assertTrue(off.getLastRotationSafe(), "footprint safety off: nothing is refused");
    assertTrue(turn > 0, "the driver's turn reaches the wheels");
  }
}
