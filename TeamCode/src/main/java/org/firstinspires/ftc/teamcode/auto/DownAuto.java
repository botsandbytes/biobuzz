package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.Nullable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import java.util.List;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Down: start at Up's start flipped across the field's middle (the line through both HIVE pivots,
 * parallel to the alliance walls), so for RED the back is on the rear wall and the right side on
 * the same TILE seam. Wait {@code auto.down_wait_sec}, then either park with one wheel in the
 * LOADING ZONE ({@code auto.down_park}: LEAVE + PARK) or only drive forward off the wall (LEAVE).
 * Never shoots.
 *
 * <p>The park guard drives to the same end pose if the deadline arrives during the wait, so a long
 * wait can never cost LEAVE (or PARK).
 */
public abstract class DownAuto extends AllianceAutoBase<config.DownAuto> {

  private List<Pose> toEnd = List.of();

  protected DownAuto(Alliance alliance) {
    super(alliance, config.DownAuto.class, "auto_poses.down");
  }

  @Override
  protected void buildPaths() {
    toEnd = sentinel.routeAround(startPose(), parkPose(), true);
  }

  @Override
  protected CommandBuilder buildAuto() {
    CommandBuilder drive =
        toEnd.isEmpty()
            ? followRoute(startPose(), sentinel.parkRoute(startPose(), parkPose()))
            : PathUtil.followAround(follower, sentinel, startPose(), toEnd, true);
    return sequential(waitMs(Math.max(0.0, config.auto.down_wait_sec) * 1000.0), drive);
  }

  /** Up's start ({@code auto_poses.up.<alliance>.start}) flipped across the field's middle. */
  @Override
  protected Pose startPose() {
    String side = alliance == Alliance.RED ? "red" : "blue";
    return flipOfUp(ConfigLoader.load(Pose.class, "auto_poses.up." + side + ".start"));
  }

  /**
   * {@code upStart} reflected across y = {@code field.size_in} / 2, the line through both HIVE
   * pivots: the audience wall swaps with the rear wall and the robot's left side with its right.
   */
  public static Pose flipOfUp(Pose upStart) {
    return PoseFactory.radians()
        .mirrorY(config.field.size_in / 2.0)
        .of(upStart.x(), upStart.y(), upStart.heading());
  }

  /** The park pose with {@code auto.down_park}, otherwise the LEAVE-only pose. */
  @Override
  protected Pose parkPose() {
    return config.auto.down_park ? poses.park : poses.leave;
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
