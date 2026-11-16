package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

/** Paths with the same linear heading behavior as the team's Pedro 2 paths. */
public final class PathUtil {
  private PathUtil() {}

  public static Path pline(Pose start, Pose end) {
    return Paths.line(start, end).linear(start, end);
  }

  public static Path pline(Pose start, Pose end, double headingEndTime) {
    return Paths.line(start, end).linear(start, end, headingEndTime);
  }

  public static Path pcurve(Pose start, Pose controlPoint, Pose end) {
    return Paths.curve(start, controlPoint, end).linear(start, end);
  }

  public static Path pcurve(Pose start, Pose controlPoint, Pose end, double headingEndTime) {
    return Paths.curve(start, controlPoint, end).linear(start, end, headingEndTime);
  }

  /**
   * One continuous path from {@code from} through {@code waypoints} with each interior corner
   * rounded into a quadratic Bezier curve: the corner is its control point, and it starts and ends
   * up to {@code radius} inches either side of the corner along the two legs (never more than 45%
   * of either leg). Straight parts lie on the original legs. A curve cuts inside its corner, so
   * each one must pass {@code clear}; a corner whose curve does not stays sharp.
   */
  public static Path smoothRoute(
      Pose from, List<Pose> waypoints, double radius, Predicate<Path> clear) {
    if (waypoints.isEmpty()) {
      throw new IllegalArgumentException("empty route");
    }
    List<Path> parts = new ArrayList<>();
    Pose cursor = from;
    Pose previous = from;
    for (int i = 0; i < waypoints.size(); i++) {
      Pose corner = waypoints.get(i);
      boolean last = i == waypoints.size() - 1;
      if (!last) {
        Pose next = waypoints.get(i + 1);
        double into = previous.distance(corner);
        double outOf = corner.distance(next);
        double r = Math.min(radius, 0.45 * Math.min(into, outOf));
        if (r > 0.5) {
          Pose in = alongLeg(previous, corner, (into - r) / into);
          Pose out = alongLeg(corner, next, r / outOf);
          Path curve = pcurve(in, corner, out);
          if (clear.test(curve)) {
            if (cursor.distance(in) > 0) {
              parts.add(pline(cursor, in));
            }
            parts.add(curve);
            cursor = out;
            previous = corner;
            continue;
          }
        }
      }
      if (cursor.distance(corner) > 0) {
        parts.add(pline(cursor, corner));
      }
      cursor = corner;
      previous = corner;
    }
    if (parts.isEmpty()) {
      parts.add(pline(from, waypoints.get(waypoints.size() - 1)));
    }
    return parts.size() == 1 ? parts.get(0) : Paths.path(parts.toArray(new Path[0]));
  }

  /**
   * The pose a fraction {@code t} of the way along the straight leg a to b, turning the short way.
   */
  private static Pose alongLeg(Pose a, Pose b, double t) {
    double turn = AngleUnit.normalizeRadians(b.heading() - a.heading());
    return new Pose(
        a.x() + t * (b.x() - a.x()), a.y() + t * (b.y() - a.y()), a.heading() + t * turn);
  }

  /**
   * Drives a {@code Sentinel.routeAround} answer as one smooth path ({@link #smoothRoute}, corners
   * rounded by {@code sentinel.route_corner_radius_in} where the rounded corner checks clear with
   * {@code sentinel}), so the robot does not stop at each corner. An empty route just stops.
   */
  public static CommandBuilder followAround(
      Follower follower,
      Sentinel sentinel,
      Pose from,
      List<Pose> waypoints,
      boolean requireAllianceSide) {
    if (waypoints.isEmpty()) {
      return instant(follower::stop).requiring(follower);
    }
    Pose end = waypoints.get(waypoints.size() - 1);
    if (waypoints.size() == 1 && from.distance(end) == 0.0) {
      return instant(() -> follower.hold(end)).requiring(follower);
    }
    double step = config.sentinel.route_sample_step;
    Path path =
        smoothRoute(
            from,
            waypoints,
            config.sentinel.route_corner_radius_in,
            curve -> sentinel.routeProblem(curve, step, requireAllianceSide).isEmpty());
    return follow(follower, path).requiring(follower);
  }

  /**
   * Drives straight legs from {@code from} through {@code waypoints} in order (a {@code
   * Sentinel.parkRoute} or {@code Sentinel.routeAround} answer); an empty route just stops. Ends
   * when the last leg does; the follower then holds the end of that path.
   */
  public static CommandBuilder followRoute(Follower follower, Pose from, List<Pose> waypoints) {
    if (waypoints.isEmpty()) {
      return instant(follower::stop).requiring(follower);
    }
    Command[] legs = new Command[waypoints.size()];
    Pose previous = from;
    for (int i = 0; i < waypoints.size(); i++) {
      Pose to = waypoints.get(i);
      legs[i] =
          previous.distance(to) == 0.0
              ? instant(() -> follower.hold(to)).requiring(follower)
              : follow(follower, pline(previous, to));
      previous = waypoints.get(i);
    }
    return sequential(legs);
  }
}
