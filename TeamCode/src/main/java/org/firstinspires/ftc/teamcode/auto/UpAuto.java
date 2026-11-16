package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.Nullable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import java.util.List;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Up: start on CELL A's side (back on the audience wall for RED, left side on the first tile seam),
 * drive to the TeleOp CELL A spot, fire at {@code teleop.manual_rev_power}, then park with one
 * wheel in the LOADING ZONE for LEAVE + PARK.
 *
 * <p>The fixed volley uses the configured pose and speed without the shot table. It only feeds
 * while CELL A is up (G417) and ends at the configured ball count or the park deadline.
 */
public abstract class UpAuto extends AllianceAutoBase<config.UpAuto> {

  private List<Pose> toShoot = List.of();
  private List<Pose> toPark = List.of();

  protected UpAuto(Alliance alliance) {
    super(alliance, config.UpAuto.class, "auto_poses.up");
  }

  @Override
  protected void buildPaths() {
    toShoot = sentinel.routeAround(startPose(), profile.shootAPose(), true);
    toPark = sentinel.routeAround(profile.shootAPose(), parkPose(), true);
  }

  @Override
  protected CommandBuilder buildAuto() {
    if (toShoot.isEmpty() || toPark.isEmpty()) {
      return followRoute(startPose(), sentinel.parkRoute(startPose(), parkPose()));
    }
    Pose shoot = profile.shootAPose();
    return sequential(
        PathUtil.followAround(follower, sentinel, startPose(), toShoot, true),
        robot.shotController.fixedVolleyCommand(
            follower,
            shoot,
            HiveCell.A,
            config.teleop.manual_rev_power,
            config.auto.balls_per_volley_count),
        idleMechanisms(),
        PathUtil.followAround(follower, sentinel, shoot, toPark, true));
  }

  @Override
  protected Pose startPose() {
    return poses.start;
  }

  @Override
  protected Pose parkPose() {
    return poses.park;
  }

  @Override
  protected @Nullable Pose firstVolleyPose() {
    return null; // A fixed shot: nothing to pre-solve.
  }
}
