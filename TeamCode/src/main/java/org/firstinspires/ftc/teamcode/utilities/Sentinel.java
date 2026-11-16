package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.stream.Collectors;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;

/**
 * BIOBUZZ footprint geometry and rule checks for one alliance, using JTS polygons.
 *
 * <p>Per-match instance owned by {@code Robot}: {@code new Sentinel(fieldModel)} fixes the alliance
 * and zones once. It answers:
 *
 * <ul>
 *   <li>What must the drive protect? The HIVE frame ({@link #getProtectedZones()}). G417 makes
 *       ramming it a foul, and Casablanca uses these zones to slow and repel the robot.
 *   <li>Is a pose legal or scoring? Fully on the alliance side (G304.A start, G402 AUTO), PARK in
 *       the LOADING ZONE, LEAVE (not touching the perimeter wall).
 *   <li>Is a whole autonomous path safe? Its swept footprint must stay inside the field, clear of
 *       the HIVE frame, and on the alliance side ({@link #routeProblem}).
 * </ul>
 */
@Configurable
public class Sentinel {
  private static final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();

  /** Footprint within this distance (inches) of a wall counts as touching it. */
  public static final double WALL_CONTACT_TOLERANCE_IN = config.sentinel.wall_contact_tolerance_in;

  private final FieldModel field;
  private final double robotWidth;
  private final List<Envelope> protectedZones;
  private final List<Geometry> routeObstacles;
  private final double passageWidth;
  private final double chassisRear;
  private final double chassisFront;
  private final double chassisHalfWidth;
  private final double wheelBehind;
  private final double wheelAhead;
  private final double wheelHalfTrack;
  private final Envelope fieldBounds;
  private final Envelope drivingBounds;
  private final Envelope routeAllianceSide;

  public Sentinel(Alliance alliance) {
    this(new FieldModel(alliance));
  }

  public Sentinel(FieldModel field) {
    this.field = field;
    this.robotWidth = config.sentinel.robot_width;
    this.protectedZones =
        config.sentinel.protect_full_hive_footprint
            ? List.of(field.hiveFrame())
            : field.hiveFrameBases();
    var chassis = config.chassis;
    chassisRear = (chassis.back_axle_from_back_in + chassis.front_axle_from_back_in) / 2.0;
    chassisFront = chassis.robot_length_in - chassisRear;
    chassisHalfWidth = chassis.robot_width_in / 2.0;
    wheelBehind = chassisRear - chassis.back_axle_from_back_in;
    wheelAhead = chassis.front_axle_from_back_in - chassisRear;
    wheelHalfTrack = chassis.wheel_track_in / 2.0;
    var hive = config.field.hive;
    boolean fitsUnderHive =
        config.sentinel.allow_under_hive_routes
            && chassis.drive_height_in > 0
            && chassis.drive_height_in <= hive.route_opening_height_in
            && chassis.drive_height_in < hive.bottom_height_in
            && hive.route_opening_width_in > 0
            && hive.route_opening_width_in < field.dimensions().frameWidthIn();
    passageWidth = fitsUnderHive ? hive.route_opening_width_in : 0.0;
    routeObstacles =
        (fitsUnderHive ? field.hivePassageSides(passageWidth) : List.of(field.hiveFrame()))
            .stream().map(GEOMETRY_FACTORY::toGeometry).collect(Collectors.toList());
    fieldBounds = field.fieldBounds();
    fieldBounds.expandBy(config.sentinel.geometry_epsilon_in);
    drivingBounds = wallClearanceBounds();
    drivingBounds.expandBy(config.sentinel.geometry_epsilon_in);
    routeAllianceSide = allianceBounds();
  }

  public FieldModel field() {
    return field;
  }

  public Alliance getAlliance() {
    return field.alliance();
  }

  public double robotWidth() {
    return robotWidth;
  }

  /**
   * {@code sentinel.footprint_safety_enabled}, read live. When false the callers that act on a
   * footprint check (the driver's turn, a shot's turn in place, TeleOp pose holds) refuse nothing.
   * The checks themselves still answer, so route planning ({@link #parkRoute}) is unchanged.
   */
  public boolean footprintSafetyEnabled() {
    return config.sentinel.footprint_safety_enabled;
  }

  // ---------------------------------------------------------------------------------------------
  // Protected zones (Casablanca)

  /** Floor regions the drive must not run into: the HIVE frame, for both alliances. */
  public List<Envelope> getProtectedZones() {
    return protectedZones;
  }

  public boolean violatesProtectedZone(Coordinate[] footprint) {
    Polygon robot = createJTSPolygon(footprint);
    for (Envelope zone : protectedZones) {
      if (robot.intersects(GEOMETRY_FACTORY.toGeometry(zone))) {
        return true;
      }
    }
    return false;
  }

  /**
   * Whether turning in place from {@code from} to {@code toHeading} (the short way) keeps the
   * footprint clear of every protected zone at every intermediate heading, sampled every degree. A
   * start already inside a zone is allowed, as in {@link #isRotationSafe}.
   */
  public boolean isRotationClear(Pose from, double toHeading) {
    return isRotationSweepClear(from, AngleUnit.normalizeRadians(toHeading - from.heading()));
  }

  private boolean isRotationSweepClear(Pose from, double turn) {
    if (violatesProtectedZone(calculateRobotFootprint(from))) {
      return true;
    }
    for (Pose p : rotationSamples(from, turn)) {
      if (violatesProtectedZone(calculateRobotFootprint(p))) {
        return false;
      }
    }
    return true;
  }

  /**
   * Checks an in-place pose assist against the HIVE, field walls and optional alliance boundary.
   */
  public Optional<String> rotationProblem(
      Pose from, double toHeading, boolean requireAllianceSide) {
    return routeProblem(
        rotationSamples(from, AngleUnit.normalizeRadians(toHeading - from.heading())),
        requireAllianceSide);
  }

  private static List<Pose> rotationSamples(Pose from, double turn) {
    int steps = Math.max(1, (int) Math.ceil(Math.abs(Math.toDegrees(turn))));
    List<Pose> samples = new ArrayList<>(steps + 1);
    for (int k = 0; k <= steps; k++) {
      samples.add(new Pose(from.x(), from.y(), from.heading() + turn * k / steps));
    }
    return samples;
  }

  /** Checks every intermediate heading in the predicted turn, allowing escape from a zone. */
  public boolean isRotationSafe(Pose currentPose, double turnInput, double lookaheadRad) {
    return isRotationSweepClear(currentPose, Math.signum(turnInput) * lookaheadRad);
  }

  // ---------------------------------------------------------------------------------------------
  // BIOBUZZ rules

  /** G304.A / G402: the whole footprint is on this alliance's side of the field. */
  public boolean isFullyOnAllianceSide(Pose pose) {
    return allianceBounds().contains(getRobotBounds(pose));
  }

  private Envelope allianceBounds() {
    Envelope side = new Envelope(field.allianceSide());
    if (field.alliance() == Alliance.RED) {
      side =
          new Envelope(
              side.getMinX(),
              side.getMaxX() - config.sentinel.center_line_margin_in,
              side.getMinY(),
              side.getMaxY());
    } else {
      side =
          new Envelope(
              side.getMinX() + config.sentinel.center_line_margin_in,
              side.getMaxX(),
              side.getMinY(),
              side.getMaxY());
    }
    side.expandBy(config.sentinel.geometry_epsilon_in);
    return side;
  }

  /** PARK: at least one wheel contact is in this alliance's LOADING ZONE. */
  public boolean isParked(Pose pose) {
    for (double depth : wheelDepthsInLoadingZone(pose)) {
      if (depth >= 0.0) {
        return true;
      }
    }
    return false;
  }

  /**
   * How far each wheel contact (back-right, front-right, front-left, back-left) is inside this
   * alliance's LOADING ZONE, in inches: positive inside, negative outside (by the largest axis
   * overshoot, never more than the true distance). Wheels sit on the axles in {@code
   * config.chassis}, {@code wheel_track_in} apart.
   */
  public double[] wheelDepthsInLoadingZone(Pose pose) {
    Envelope zone = field.loadingZone();
    Coordinate[] wheels = calculateFootprint(pose, wheelBehind, wheelAhead, wheelHalfTrack);
    double[] depths = new double[wheels.length];
    for (int i = 0; i < wheels.length; i++) {
      Coordinate w = wheels[i];
      depths[i] =
          Math.min(
              Math.min(w.x - zone.getMinX(), zone.getMaxX() - w.x),
              Math.min(w.y - zone.getMinY(), zone.getMaxY() - w.y));
    }
    return depths;
  }

  /** The footprint touches (or crosses) a perimeter wall. LEAVE requires this to be false. */
  public boolean isTouchingWall(Pose pose) {
    Envelope inner = new Envelope(field.fieldBounds());
    inner.expandBy(-config.sentinel.wall_contact_tolerance_in);
    return !inner.contains(getRobotBounds(pose));
  }

  /** End-of-AUTO scoring: LEAVE (not touching the wall) and PARK both earned at this pose. */
  public boolean earnsLeaveAndPark(Pose pose) {
    return isParked(pose) && !isTouchingWall(pose);
  }

  /** The footprint is clear of the HIVE frame (whatever {@link #getProtectedZones()} covers). */
  public boolean isClearOfHive(Pose pose) {
    return !violatesProtectedZone(calculateRobotFootprint(pose));
  }

  /**
   * Checks the swept chassis for wall and HIVE-support clearance, and optionally for staying on its
   * alliance side. A wall-touching start can leave the wall at a fixed heading.
   *
   * @return a description of the first violation, or empty if the path is clear
   */
  public Optional<String> routeProblem(Path path, double stepT, boolean requireAllianceSide) {
    List<Pose> samples = new ArrayList<>();
    for (double t = 0.0; t < 1.0; t += stepT) {
      samples.add(path.get(t));
    }
    samples.add(path.get(1.0));
    return routeProblem(samples, requireAllianceSide);
  }

  /** {@link #routeProblem(Path, double, boolean)} over already-sampled poses. */
  public Optional<String> routeProblem(List<Pose> samples, boolean requireAllianceSide) {
    Envelope allowed = new Envelope(drivingBounds);
    Pose start = samples.isEmpty() ? null : samples.get(0);
    if (start != null && ShotInputs.isFinitePose(start)) {
      allowed.expandToInclude(chassisBounds(start));
    }
    Envelope bounds = new Envelope(drivingBounds);
    for (Pose p : samples) {
      if (!ShotInputs.isFinitePose(p)) {
        return Optional.of("invalid route pose");
      }
      Coordinate[] footprint = calculateFootprint(p, chassisRear, chassisFront, chassisHalfWidth);
      bounds = getProjectedBounds(footprint);
      if (!fieldBounds.contains(bounds)) {
        return Optional.of("leaves the field at " + where(p));
      }
      if (!drivingBounds.contains(bounds)
          && (!allowed.contains(bounds)
              || (start != null
                  && Math.abs(AngleUnit.normalizeRadians(p.heading() - start.heading()))
                      > config.sentinel.geometry_epsilon_in))) {
        return Optional.of("too close to a wall at " + where(p));
      }
      // A wall-touching start may slide inward. It cannot turn beside a wall, move closer to it,
      // or return once it has cleared it.
      allowed.init(drivingBounds);
      allowed.expandToInclude(bounds);
      if (hitsRouteObstacle(footprint, bounds)) {
        return Optional.of("hits the HIVE frame at " + where(p));
      }
      if (requireAllianceSide && !routeAllianceSide.contains(bounds)) {
        return Optional.of("crosses the centre line (G402) at " + where(p));
      }
    }
    if (!drivingBounds.contains(bounds)) {
      return Optional.of("route ends too close to a wall");
    }
    return Optional.empty();
  }

  private boolean hitsRouteObstacle(Coordinate[] footprint, Envelope bounds) {
    for (Geometry obstacle : routeObstacles) {
      if (obstacle.getEnvelopeInternal().intersects(bounds)
          && obstacle.intersects(createJTSPolygon(footprint))) {
        return true;
      }
    }
    return false;
  }

  private static String where(Pose p) {
    return String.format(Locale.ROOT, "(%.1f, %.1f)", p.x(), p.y());
  }

  /**
   * Waypoints from {@code here} to {@code park} whose straight segments keep the swept footprint in
   * the field, clear of the HIVE frame and on the alliance side. Tries the direct line first, then
   * detours through a lane beside the HIVE on the alliance's side. Used when AUTO's park reserve
   * cancels the routine wherever the robot happens to be.
   *
   * @return poses to follow in order, ending at {@code park}; an empty list if nothing checks clear
   */
  public List<Pose> parkRoute(Pose here, Pose park) {
    Optional<Pose> measured = routeStart(here);
    if (measured.isEmpty() || !isClearOfWalls(park)) {
      return List.of();
    }
    Pose start = measured.get();
    for (List<Pose> candidate : candidateRoutes(start, park)) {
      if (segmentsClear(start, candidate)) {
        return candidate;
      }
    }
    return routeAround(start, park, true);
  }

  /**
   * Direct; via the HIVE-side lane at this y; along the lane to the park y. The last two also come
   * in a variant that reaches the lane before turning, for a robot pushed close to the HIVE frame,
   * where turning in place would sweep a corner into it.
   */
  private List<List<Pose>> candidateRoutes(Pose from, Pose park) {
    double laneRedX = field.hiveFrame().getMinX() - detourClearance();
    Pose fromRed = field.fromRedFrame(from);
    Pose parkRed = field.fromRedFrame(park);
    Pose besideFrom = field.fromRedFrame(new Pose(laneRedX, fromRed.y(), parkRed.heading()));
    Pose besidePark = field.fromRedFrame(new Pose(laneRedX, parkRed.y(), parkRed.heading()));
    Pose laneBeforeTurning = field.fromRedFrame(new Pose(laneRedX, fromRed.y(), fromRed.heading()));
    return List.of(
        List.of(park),
        List.of(besideFrom, park),
        List.of(besideFrom, besidePark, park),
        List.of(laneBeforeTurning, park),
        List.of(laneBeforeTurning, besidePark, park));
  }

  /** Inches between the HIVE-side lane used by {@link #parkRoute} and the robot footprint. */
  public static final double PARK_LANE_MARGIN_IN = config.sentinel.park_lane_margin_in;

  private boolean segmentsClear(Pose start, List<Pose> waypoints) {
    Pose from = start;
    for (Pose to : waypoints) {
      if (!segmentClear(from, to, true)) {
        return false;
      }
      from = to;
    }
    return true;
  }

  /** Checks a straight leg at intervals no larger than an inch or one degree of turn. */
  private boolean segmentClear(Pose from, Pose to, boolean requireAllianceSide) {
    List<Pose> samples = new ArrayList<>();
    double turn = AngleUnit.normalizeRadians(to.heading() - from.heading());
    int steps =
        Math.max(2, (int) Math.ceil(Math.max(from.distance(to), Math.abs(Math.toDegrees(turn)))));
    samples.add(from);
    for (int i = 1; i < steps; i++) {
      double t = (double) i / steps;
      // Same linear heading interpolation PathUtil.pline gives the follower.
      samples.add(
          new Pose(
              from.x() + t * (to.x() - from.x()),
              from.y() + t * (to.y() - from.y()),
              from.heading() + t * turn));
    }
    samples.add(to);
    return routeProblem(samples, requireAllianceSide).isEmpty();
  }

  /** {@code here} slid straight off any wall it is too close to turn at, keeping its heading. */
  private Pose offWall(Pose here) {
    double clearance =
        Math.min(detourClearance() + config.sentinel.route_wall_clearance_in, field.size() / 2.0);
    double size = field.size();
    return new Pose(
        Math.clamp(here.x(), clearance, size - clearance),
        Math.clamp(here.y(), clearance, size - clearance),
        here.heading());
  }

  /** Maximum distance from the spin centre to a chassis corner, plus lane margin. */
  private double detourClearance() {
    return Math.hypot(Math.max(chassisFront, chassisRear), chassisHalfWidth)
        + config.sentinel.park_lane_margin_in;
  }

  /**
   * Shortest clear route among the direct leg, the HIVE opening and the configured detour
   * waypoints, using straight segments with interpolated headings. The robot can slide inward from
   * a wall-touching start before turning. The destination must have wall clearance. Plan once per
   * trip on the Control Hub. {@code here} is a measured pose: one that reads slightly past a wall
   * is planned from the wall ({@link #routeStart}).
   *
   * @return poses to drive through in order, ending at {@code to}; empty if no route checks clear
   */
  public List<Pose> routeAround(Pose here, Pose to, boolean requireAllianceSide) {
    Optional<Pose> measured = routeStart(here);
    if (measured.isEmpty() || !isClearOfWalls(to)) {
      return List.of();
    }
    Pose from = measured.get();
    List<Pose> route = shortestDetour(from, to, requireAllianceSide);
    if (!route.isEmpty()) {
      return route;
    }
    Pose departure = offWall(from);
    if (departure.distance(from) == 0.0 || !segmentClear(from, departure, requireAllianceSide)) {
      return List.of();
    }
    List<Pose> rest = shortestDetour(departure, to, requireAllianceSide);
    if (rest.isEmpty()) {
      return rest;
    }
    List<Pose> withDeparture = new ArrayList<>(rest.size() + 1);
    withDeparture.add(departure);
    withDeparture.addAll(rest);
    return withDeparture;
  }

  /** The physical chassis is contained within the perimeter, including a legal start at a wall. */
  public boolean isInsideField(Pose pose) {
    return ShotInputs.isFinitePose(pose) && fieldBounds.contains(chassisBounds(pose));
  }

  /**
   * A measured route start, moved straight inward (heading kept) just far enough that the chassis
   * is inside the field. A robot pushed against a wall reads slightly past it: the Pinpoint returns
   * a set pose rounded to float (the reset corner comes back 5e-7 in out) and heading noise swings
   * the chassis corners. Empty if the chassis reads more than {@code
   * sentinel.route_start_tolerance_in} past a wall, where the pose is not to be trusted.
   */
  public Optional<Pose> routeStart(Pose here) {
    if (!ShotInputs.isFinitePose(here)) {
      return Optional.empty();
    }
    Envelope bounds = chassisBounds(here);
    Envelope limits = field.fieldBounds();
    if (bounds.getWidth() > limits.getWidth() || bounds.getHeight() > limits.getHeight()) {
      return Optional.empty();
    }
    double dx =
        Math.clamp(0.0, limits.getMinX() - bounds.getMinX(), limits.getMaxX() - bounds.getMaxX());
    double dy =
        Math.clamp(0.0, limits.getMinY() - bounds.getMinY(), limits.getMaxY() - bounds.getMaxY());
    if (Math.max(Math.abs(dx), Math.abs(dy)) > config.sentinel.route_start_tolerance_in) {
      return Optional.empty();
    }
    return Optional.of(
        dx == 0.0 && dy == 0.0 ? here : new Pose(here.x() + dx, here.y() + dy, here.heading()));
  }

  private boolean isClearOfWalls(Pose pose) {
    return ShotInputs.isFinitePose(pose) && drivingBounds.contains(chassisBounds(pose));
  }

  /** Move a wall-side practice stop inward just enough to preserve the driving clearance. */
  public Pose wallStandoff(Pose pose) {
    Envelope bounds = chassisBounds(pose);
    Envelope limits = wallClearanceBounds();
    double dx =
        Math.clamp(0.0, limits.getMinX() - bounds.getMinX(), limits.getMaxX() - bounds.getMaxX());
    double dy =
        Math.clamp(0.0, limits.getMinY() - bounds.getMinY(), limits.getMaxY() - bounds.getMaxY());
    return new Pose(pose.x() + dx, pose.y() + dy, pose.heading());
  }

  private Envelope wallClearanceBounds() {
    Envelope bounds = field.fieldBounds();
    bounds.expandBy(-config.sentinel.route_wall_clearance_in);
    return bounds;
  }

  /** Dijkstra from {@code from} to {@code to} over the detour points, checking legs lazily. */
  private List<Pose> shortestDetour(Pose from, Pose to, boolean requireAllianceSide) {
    if (segmentClear(from, to, requireAllianceSide)) {
      return List.of(to);
    }
    if (from.distance(to) == 0.0 || !segmentClear(to, to, requireAllianceSide)) {
      return List.of();
    }
    Envelope frame = field.hiveFrame();
    double c = detourClearance();
    double minX = frame.getMinX() - c;
    double maxX = frame.getMaxX() + c;
    double minY = frame.getMinY() - c;
    double maxY = frame.getMaxY() + c;
    double h = to.heading();
    List<Pose> nodes = new ArrayList<>();
    nodes.add(from);
    nodes.add(new Pose(minX, minY, h));
    nodes.add(new Pose(maxX, minY, h));
    nodes.add(new Pose(maxX, maxY, h));
    nodes.add(new Pose(minX, maxY, h));
    for (Pose p : List.of(from, to)) {
      nodes.add(new Pose(p.x(), minY, h));
      nodes.add(new Pose(p.x(), maxY, h));
      nodes.add(new Pose(minX, p.y(), h));
      nodes.add(new Pose(maxX, p.y(), h));
    }
    if (passageWidth > 0) {
      nodes.add(new Pose(field.center(), minY, h));
      nodes.add(new Pose(field.center(), maxY, h));
    }
    if (from.heading() != h) {
      // Retreat or enter the opening at the current heading before starting a wider turn.
      int detours = nodes.size();
      for (int i = 1; i < detours; i++) {
        Pose p = nodes.get(i);
        nodes.add(new Pose(p.x(), p.y(), from.heading()));
      }
    }
    nodes.add(to);
    int n = nodes.size();
    int goal = n - 1;
    double[] dist = new double[n];
    int[] previous = new int[n];
    boolean[] settled = new boolean[n];
    java.util.Arrays.fill(dist, Double.POSITIVE_INFINITY);
    java.util.Arrays.fill(previous, -1);
    dist[0] = 0.0;
    while (true) {
      int u = -1;
      for (int i = 0; i < n; i++) {
        if (!settled[i] && dist[i] < Double.POSITIVE_INFINITY && (u < 0 || dist[i] < dist[u])) {
          u = i;
        }
      }
      if (u < 0 || u == goal) {
        break;
      }
      settled[u] = true;
      for (int v = 1; v < n; v++) {
        if (settled[v]) {
          continue;
        }
        double leg = nodes.get(u).distance(nodes.get(v));
        double d = dist[u] + leg;
        if (leg > 0.0
            && d < dist[v]
            && segmentClear(nodes.get(u), nodes.get(v), requireAllianceSide)) {
          dist[v] = d;
          previous[v] = u;
        }
      }
    }
    if (previous[goal] < 0) {
      return List.of();
    }
    List<Pose> route = new ArrayList<>();
    for (int v = goal; v != 0; v = previous[v]) {
      route.add(0, nodes.get(v));
    }
    return route;
  }

  // ---------------------------------------------------------------------------------------------
  // Footprint

  public Envelope getRobotBounds(Pose pose) {
    return getProjectedBounds(calculateRobotFootprint(pose));
  }

  public Coordinate[] calculateRobotFootprint(Pose pose) {
    return calculateFootprint(pose, robotWidth);
  }

  private Envelope chassisBounds(Pose pose) {
    return getProjectedBounds(
        calculateFootprint(pose, chassisRear, chassisFront, chassisHalfWidth));
  }

  private static Coordinate[] calculateFootprint(Pose pose, double width) {
    double radius = width / 2.0;
    return calculateFootprint(pose, radius, radius, radius);
  }

  private static Coordinate[] calculateFootprint(
      Pose pose, double rear, double front, double halfWidth) {
    double heading = pose.heading();
    double centerX = pose.x();
    double centerY = pose.y();
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    double[] xOffsets = {-rear, front, front, -rear};
    double[] yOffsets = {-halfWidth, -halfWidth, halfWidth, halfWidth};
    Coordinate[] corners = new Coordinate[4];
    for (int i = 0; i < 4; i++) {
      double rotatedX = (xOffsets[i] * cos) - (yOffsets[i] * sin);
      double rotatedY = (xOffsets[i] * sin) + (yOffsets[i] * cos);
      corners[i] = new Coordinate(centerX + rotatedX, centerY + rotatedY);
    }
    return corners;
  }

  private static Envelope getProjectedBounds(Coordinate[] footprint) {
    Envelope bounds = new Envelope();
    for (Coordinate p : footprint) {
      bounds.expandToInclude(p);
    }
    return bounds;
  }

  private static Polygon createJTSPolygon(Coordinate[] points) {
    if (points == null || points.length == 0) {
      return GEOMETRY_FACTORY.createPolygon();
    }
    Coordinate[] coords = new Coordinate[points.length + 1];
    for (int i = 0; i < points.length; i++) {
      coords[i] = new Coordinate(points[i].x, points[i].y);
    }
    coords[points.length] = new Coordinate(points[0].x, points[0].y);
    return GEOMETRY_FACTORY.createPolygon(coords);
  }
}
