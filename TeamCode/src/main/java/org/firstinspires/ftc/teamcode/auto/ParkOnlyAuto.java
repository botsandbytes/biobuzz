package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;

import androidx.annotation.Nullable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Start in the corner of the alliance wall and the far wall, next to the LOADING ZONE, and park
 * with one wheel in it. No shot. Worth LEAVE (3) + PARK (5).
 */
// Paths are assigned by buildPaths(), which init() calls, not by a constructor.
@SuppressWarnings("NullAway.Init")
public abstract class ParkOnlyAuto extends AllianceAutoBase<config.ParkOnlyAuto> {

  private Path toPark;

  protected ParkOnlyAuto(Alliance alliance) {
    super(alliance, config.ParkOnlyAuto.class, "auto_poses.park_only");
  }

  @Override
  protected void buildPaths() {
    toPark = pline(poses.start, poses.park);
  }

  @Override
  protected CommandBuilder buildAuto() {
    return sequential(follow(follower, toPark));
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
    return null;
  }

  @Override
  protected boolean startsWithPreload() {
    return false;
  }
}
