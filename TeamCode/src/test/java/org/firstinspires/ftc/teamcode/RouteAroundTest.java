package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Test;

/**
 * Generated routes keep the chassis clear of the supports and field walls, including when they pass
 * under the HIVE or depart from a wall-touching start.
 */
public class RouteAroundTest {
  private final Sentinel red = new Sentinel(Alliance.RED);
  private final MatchProfile redProfile = config.loadMatchProfile(Alliance.RED);
  private final MatchProfile blueProfile = config.loadMatchProfile(Alliance.BLUE);

  /** Samples every inch along each straight leg, heading turning evenly the short way. */
  private static List<Pose> sampled(Pose start, List<Pose> route) {
    List<Pose> samples = new ArrayList<>();
    Pose from = start;
    for (Pose to : route) {
      int steps = Math.max(2, (int) Math.ceil(from.distance(to)));
      double turn = AngleUnit.normalizeRadians(to.heading() - from.heading());
      for (int i = 0; i <= steps; i++) {
        double t = (double) i / steps;
        samples.add(
            new Pose(
                from.x() + t * (to.x() - from.x()),
                from.y() + t * (to.y() - from.y()),
                from.heading() + t * turn));
      }
      from = to;
    }
    return samples;
  }

  private static double headingGap(Pose a, Pose b) {
    return Math.abs(AngleUnit.normalizeRadians(a.heading() - b.heading()));
  }

  private void assertClearRoute(
      Sentinel sentinel, Pose from, Pose to, List<Pose> route, boolean allianceSide) {
    assertFalse(route.isEmpty(), "no route from " + from + " to " + to);
    Pose end = route.get(route.size() - 1);
    assertEquals(to.x(), end.x(), 1e-9);
    assertEquals(to.y(), end.y(), 1e-9);
    assertEquals(0.0, headingGap(to, end), 1e-9, "ends on its heading");

    List<Pose> samples = sampled(from, route);
    assertTrue(
        sentinel.routeProblem(samples, allianceSide).isEmpty(),
        () -> from + " -> " + to + ": " + sentinel.routeProblem(samples, allianceSide).get());
  }

  /** The Point To Point Drive Test's tour: every TeleOp pose of both alliances, from the corner. */
  @Test
  public void theTourReachesEveryTeleOpPoseOfBothAlliancesAroundTheHive() {
    List<Pose> tour =
        List.of(
            redProfile.resetPose(),
            redProfile.shootBPose(),
            redProfile.parkPose(),
            redProfile.flowerPose(),
            redProfile.shootAPose(),
            redProfile.startPose(),
            blueProfile.shootBPose(),
            blueProfile.parkPose(),
            blueProfile.flowerPose(),
            blueProfile.shootAPose(),
            blueProfile.startPose(),
            redProfile.resetPose());
    for (int i = 0; i + 1 < tour.size(); i++) {
      Pose from = i == 0 ? tour.get(i) : red.wallStandoff(tour.get(i));
      Pose to = red.wallStandoff(tour.get(i + 1));
      assertClearRoute(red, from, to, red.routeAround(from, to, false), false);
    }
  }

  /** A pose as the Pinpoint returns it after {@code setPose}: stored as float mm and radians. */
  private static Pose pinpointReading(Pose set) {
    return new Pose(
        (float) (set.x() * 25.4) / 25.4, (float) (set.y() * 25.4) / 25.4, (float) set.heading());
  }

  /**
   * On the robot a route starts from the measured pose. Zeroed in the reset corner, it reads a hair
   * past both walls (float rounding, heading noise), which once refused every stop of the tour.
   * Each stop must still plan, from the corner moved straight in; a reading farther out than the
   * tolerance plans nothing.
   */
  @Test
  public void aStartReadSlightlyPastTheWallsStillReachesEveryStop() {
    Pose corner = redProfile.resetPose();
    Pose pinpoint = pinpointReading(corner);
    assertFalse(red.isInsideField(pinpoint), "the read-back corner pokes past the walls");
    double tolerance = config.sentinel.route_start_tolerance_in;
    double noise = Math.toRadians(0.5);
    List<Pose> stops =
        List.of(
            redProfile.shootBPose(),
            redProfile.parkPose(),
            redProfile.flowerPose(),
            redProfile.shootAPose(),
            redProfile.startPose(),
            blueProfile.shootBPose(),
            blueProfile.parkPose(),
            blueProfile.flowerPose(),
            blueProfile.shootAPose(),
            blueProfile.startPose());
    for (Pose here :
        List.of(
            pinpoint,
            new Pose(corner.x(), corner.y(), corner.heading() + noise),
            new Pose(corner.x() - 0.3, corner.y() - 0.3, corner.heading() - noise))) {
      Pose start = red.routeStart(here).orElseThrow();
      assertTrue(red.isInsideField(start), () -> here + " moved in to " + start);
      assertEquals(0.0, headingGap(here, start), 1e-12, "slides in without turning");
      assertTrue(start.distance(here) <= Math.hypot(tolerance, tolerance));
      for (Pose stop : stops) {
        Pose to = red.wallStandoff(stop);
        assertClearRoute(red, start, to, red.routeAround(here, to, false), false);
      }
      Pose park = redProfile.parkPose();
      assertClearRoute(red, start, park, red.parkRoute(here, park), true);
    }
    Pose lost = new Pose(corner.x() - tolerance - 0.5, corner.y(), corner.heading());
    assertTrue(red.routeStart(lost).isEmpty(), "a reading this far out is not trusted");
    assertTrue(red.routeAround(lost, red.wallStandoff(redProfile.shootAPose()), false).isEmpty());
    assertTrue(red.parkRoute(lost, redProfile.parkPose()).isEmpty());
  }

  @Test
  public void fromTheResetCornerToEveryRedPoseStayingOnTheAllianceSide() {
    Pose corner = redProfile.resetPose();
    for (Pose to :
        List.of(
            redProfile.shootAPose(),
            redProfile.shootBPose(),
            redProfile.parkPose(),
            redProfile.flowerPose(),
            redProfile.startPose())) {
      Pose destination = red.wallStandoff(to);
      assertClearRoute(red, corner, destination, red.routeAround(corner, destination, true), true);
    }
  }

  /** Driver D-pad: every FLOWER is reachable around the HIVE from both shooting spots and park. */
  @Test
  public void everyFlowerIsReachableFromTheShootingSpotsAndPark() {
    for (MatchProfile profile : List.of(redProfile, blueProfile)) {
      Sentinel sentinel = new Sentinel(profile.alliance());
      for (Pose flower : sentinel.field().flowerApproaches(profile.flowerPose())) {
        for (Pose from : List.of(profile.shootAPose(), profile.shootBPose(), profile.parkPose())) {
          List<Pose> route = sentinel.routeAround(from, flower, true);
          boolean allianceSide = !route.isEmpty();
          if (!allianceSide) {
            route = sentinel.routeAround(from, flower, false);
          }
          assertClearRoute(sentinel, from, flower, route, allianceSide);
        }
      }
    }
  }

  @Test
  public void shootAToShootBGoesAroundTheHiveNotThroughIt() {
    Pose a = redProfile.shootAPose();
    Pose b = redProfile.shootBPose();
    assertFalse(
        red.routeProblem(sampled(a, List.of(b)), true).isEmpty(),
        "the straight line crosses the HIVE frame, so a detour is needed");
    List<Pose> route = red.routeAround(a, b, true);
    assertClearRoute(red, a, b, route, true);
    assertTrue(route.size() >= 2, "went around through at least one detour point");
  }

  @Test
  public void thePlannerCanDriveStraightThroughTheOpeningBetweenTheSupports() {
    double x = red.field().center();
    Pose from = new Pose(x, 35, Math.PI / 2);
    Pose to = new Pose(x, 105, Math.PI / 2);
    assertEquals(List.of(to), red.routeAround(from, to, false));
    assertTrue(red.routeAround(from, to, true).isEmpty(), "AUTO must stay on its alliance side");
  }

  @Test
  public void thePlannerRejectsSupportContactAndKeepsTallRobotsOutsideTheOpening() {
    double x = red.field().center();
    double support = x - config.field.hive.route_opening_width_in / 2;
    double nearSupport = support + config.chassis.robot_width_in / 2 - 0.1;
    Pose start = new Pose(nearSupport, 35, Math.PI / 2);
    Pose hit = new Pose(nearSupport, red.field().center(), Math.PI / 2);
    assertTrue(red.routeAround(start, hit, false).isEmpty(), "the chassis hits the side support");

    config.chassis.drive_height_in = config.field.hive.route_opening_height_in + 1.0;
    Sentinel tall = new Sentinel(Alliance.RED);
    assertTrue(
        tall.routeAround(start, new Pose(x, red.field().center(), Math.PI / 2), false).isEmpty());
  }

  @Test
  public void aWallTouchingStartCanDepartButThePlannerRefusesToReturnToTheWall() {
    Pose wall = redProfile.resetPose();
    Pose requested = new Pose(20, 25, 0);
    List<Pose> departure = red.routeAround(wall, requested, true);
    assertClearRoute(red, wall, requested, departure, true);
    assertEquals(0.0, headingGap(wall, departure.get(0)), 1e-9);
    assertTrue(red.routeAround(requested, wall, true).isEmpty());
    assertTrue(red.parkRoute(requested, wall).isEmpty());
    Pose standoff = red.wallStandoff(wall);
    double clearance = config.sentinel.route_wall_clearance_in;
    assertEquals(wall.x() + clearance, standoff.x(), 1e-6);
    assertEquals(wall.y() + clearance, standoff.y(), 1e-6);
    assertClearRoute(red, requested, standoff, red.routeAround(requested, standoff, true), true);
  }

  @Test
  public void everyPerimeterWallBlocksAChassisOverhangEvenWithThePoseInsideTheField() {
    double center = red.field().center();
    double size = red.field().size();
    Pose start = new Pose(center, 30, 0);
    for (Pose end :
        List.of(
            new Pose(5, center, 0),
            new Pose(size - 5, center, 0),
            new Pose(center, 5, 0),
            new Pose(center, size - 5, 0))) {
      assertTrue(red.routeAround(start, end, false).isEmpty());
    }
  }

  @Test
  public void aClearStraightLineIsDrivenDirectly() {
    Pose from = new Pose(20, 20, 0);
    Pose to = new Pose(30, 25, Math.PI / 2);
    assertEquals(List.of(to), red.routeAround(from, to, true));
  }

  @Test
  public void aGoalTouchingTheHiveHasNoRoute() {
    FieldModel field = red.field();
    Pose inside = new Pose(field.hiveFrame().getMinX(), field.center(), 0);
    assertTrue(red.routeAround(redProfile.resetPose(), inside, false).isEmpty());
  }

  /** The field is a half-turn: BLUE's route between rotated poses is RED's route, rotated. */
  @Test
  public void blueRoutesAreRedRoutesRotated() {
    Sentinel blue = new Sentinel(Alliance.BLUE);
    FieldModel blueField = blue.field();
    Pose a = redProfile.shootAPose();
    Pose b = redProfile.shootBPose();
    List<Pose> redRoute = red.routeAround(a, b, true);
    List<Pose> blueRoute =
        blue.routeAround(blueField.fromRedFrame(a), blueField.fromRedFrame(b), true);
    assertEquals(redRoute.size(), blueRoute.size());
    for (int i = 0; i < redRoute.size(); i++) {
      Pose expected = blueField.fromRedFrame(redRoute.get(i));
      assertEquals(expected.x(), blueRoute.get(i).x(), 1e-6);
      assertEquals(expected.y(), blueRoute.get(i).y(), 1e-6);
      assertEquals(0.0, headingGap(expected, blueRoute.get(i)), 1e-6);
    }
  }

  /** Soundness over a grid of starts: whatever the planner answers, every leg is clear. */
  @Test
  public void everyPlannedRouteFromAGridOfStartsIsClear() {
    Pose to = redProfile.shootBPose();
    int planned = 0;
    for (double x = 15; x <= 125; x += 22) {
      for (double y = 15; y <= 125; y += 22) {
        Pose from = new Pose(x, y, Math.toRadians(x + y));
        List<Pose> route = red.routeAround(from, to, false);
        if (!route.isEmpty()) {
          planned++;
          assertClearRoute(red, from, to, route, false);
        }
      }
    }
    assertTrue(planned >= 20, "most of the grid can reach shoot_b, got " + planned);
  }

  private static double sampledLength(java.util.function.DoubleFunction<Pose> at) {
    double length = 0.0;
    Pose previous = at.apply(0.0);
    for (int i = 1; i <= 2000; i++) {
      Pose p = at.apply(i / 2000.0);
      length += previous.distance(p);
      previous = p;
    }
    return length;
  }

  /** The driven go-around: corners rounded into curves that stay clear, one path to the goal. */
  @Test
  public void roundedCornersStayClearEndAtTheGoalAndShortenTheDrive() {
    Pose a = redProfile.shootAPose();
    Pose b = redProfile.shootBPose();
    List<Pose> route = red.routeAround(a, b, true);
    double step = config.sentinel.route_sample_step;
    com.pedropathing.paths.Path smooth =
        org.firstinspires.ftc.teamcode.auto.PathUtil.smoothRoute(
            a,
            route,
            config.sentinel.route_corner_radius_in,
            curve -> red.routeProblem(curve, step, true).isEmpty());
    assertTrue(
        red.routeProblem(smooth, step, true).isEmpty(),
        () -> red.routeProblem(smooth, step, true).get());
    Pose end = smooth.get(1.0);
    assertEquals(b.x(), end.x(), 1e-6);
    assertEquals(b.y(), end.y(), 1e-6);
    // Independent check that the corners really were rounded: a rounded route is shorter.
    List<Pose> polyline = sampled(a, route);
    double sharp = 0.0;
    for (int i = 1; i < polyline.size(); i++) {
      sharp += polyline.get(i - 1).distance(polyline.get(i));
    }
    assertTrue(sampledLength(smooth::get) < sharp - 1.0, "rounded corners shorten the drive");
  }

  @Test
  public void aCornerWhoseCurveIsNotClearStaysSharp() {
    Pose a = redProfile.shootAPose();
    Pose b = redProfile.shootBPose();
    List<Pose> route = red.routeAround(a, b, true);
    com.pedropathing.paths.Path sharp =
        org.firstinspires.ftc.teamcode.auto.PathUtil.smoothRoute(a, route, 12.0, curve -> false);
    for (Pose corner : route) {
      double nearest = Double.POSITIVE_INFINITY;
      for (int i = 0; i <= 4000; i++) {
        nearest = Math.min(nearest, sharp.get(i / 4000.0).distance(corner));
      }
      assertTrue(nearest < 0.5, "passes through " + corner);
    }
  }
}
