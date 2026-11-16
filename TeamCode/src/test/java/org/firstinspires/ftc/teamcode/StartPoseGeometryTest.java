package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.util.AffineTransformation;

/**
 * Places the real chassis outline ({@code config.chassis}) at every start pose and at the TeleOp
 * corner reset pose. Pedro's pose is the spin centre, midway between the axles, so the outline is
 * not centred on it: this test, not Sentinel's square footprint, says whether those poses put the
 * robot where the drivers place it. Tagged {@code push-config}: the robot never re-checks them.
 */
@Tag("push-config")
public class StartPoseGeometryTest {
  private static final double EPS = 1e-6;
  private static final GeometryFactory GF = new GeometryFactory();

  private enum Wall {
    RED_ALLIANCE,
    BLUE_ALLIANCE,
    AUDIENCE,
    REAR
  }

  /** Spin centre to back edge: midway between the axles, both measured from the back edge. */
  private static double back() {
    var c = config.chassis;
    return (c.back_axle_from_back_in + c.front_axle_from_back_in) / 2.0;
  }

  private static double front() {
    return config.chassis.robot_length_in - back();
  }

  private static double half() {
    return config.chassis.robot_width_in / 2.0;
  }

  /** A point {@code forward} ahead of and {@code left} of the spin centre, in field coordinates. */
  private static Coordinate at(Pose pose, double forward, double left) {
    AffineTransformation toField =
        AffineTransformation.rotationInstance(pose.heading()).translate(pose.x(), pose.y());
    return toField.transform(new Coordinate(forward, left), new Coordinate());
  }

  private static Polygon outline(Pose p) {
    return GF.createPolygon(
        new Coordinate[] {
          at(p, -back(), -half()),
          at(p, front(), -half()),
          at(p, front(), half()),
          at(p, -back(), half()),
          at(p, -back(), -half())
        });
  }

  /** The perimeter wall both points lie on, if any. */
  private static Optional<Wall> wallUnder(Coordinate a, Coordinate b) {
    double s = config.field.size_in;
    if (Math.abs(a.x) < EPS && Math.abs(b.x) < EPS) {
      return Optional.of(Wall.RED_ALLIANCE);
    }
    if (Math.abs(a.x - s) < EPS && Math.abs(b.x - s) < EPS) {
      return Optional.of(Wall.BLUE_ALLIANCE);
    }
    if (Math.abs(a.y) < EPS && Math.abs(b.y) < EPS) {
      return Optional.of(Wall.AUDIENCE);
    }
    if (Math.abs(a.y - s) < EPS && Math.abs(b.y - s) < EPS) {
      return Optional.of(Wall.REAR);
    }
    return Optional.empty();
  }

  private static Optional<Wall> backEdgeWall(Pose p) {
    return wallUnder(at(p, -back(), -half()), at(p, -back(), half()));
  }

  private static Optional<Wall> leftEdgeWall(Pose p) {
    return wallUnder(at(p, -back(), half()), at(p, front(), half()));
  }

  private static void assertInsideField(String name, Polygon outline) {
    Envelope field = new FieldModel(Alliance.RED).fieldBounds();
    field.expandBy(EPS);
    assertTrue(field.contains(outline.getEnvelopeInternal()), name + " inside the field");
  }

  @Test
  public void startPosesPutTheBackEdgeOnAWallOnTheAllianceSide() {
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      List<String> names = new ArrayList<>();
      List<Pose> starts = new ArrayList<>();
      for (var routine : AutoFixtures.routines(alliance)) {
        names.add(routine + " start");
        starts.add(routine.start());
      }
      names.add(alliance + " TeleOp start");
      starts.add(MatchProfile.loadMatchProfile(alliance).startPose());

      for (int i = 0; i < starts.size(); i++) {
        String name = names.get(i);
        Pose start = starts.get(i);
        Polygon robot = outline(start);
        assertTrue(backEdgeWall(start).isPresent(), name + " has its back edge on a wall");
        assertInsideField(name, robot);
        Envelope side = field.allianceSide();
        side.expandBy(EPS);
        assertTrue(side.contains(robot.getEnvelopeInternal()), name + " on the alliance side");
        assertTrue(
            robot.intersection(GF.toGeometry(field.loadingZone())).getArea() < EPS,
            name + " outside the LOADING ZONE");
        assertFalse(
            robot.intersects(GF.toGeometry(field.hiveFrame())), name + " clear of the HIVE frame");
      }
    }
  }

  @Test
  public void resetPoseIsTheAudienceWallAndRedWallCornerInEveryMatch() {
    Pose red = MatchProfile.loadMatchProfile(Alliance.RED).resetPose();
    Pose blue = MatchProfile.loadMatchProfile(Alliance.BLUE).resetPose();
    assertEquals(red.x(), blue.x(), EPS, "BLUE resets in the same corner");
    assertEquals(red.y(), blue.y(), EPS, "BLUE resets in the same corner");
    assertEquals(red.heading(), blue.heading(), EPS, "BLUE resets facing the same way");

    assertEquals(Optional.of(Wall.AUDIENCE), backEdgeWall(red), "back on the audience wall");
    assertEquals(Optional.of(Wall.RED_ALLIANCE), leftEdgeWall(red), "left side on the RED wall");
    assertInsideField("reset pose", outline(red));

    Coordinate here = new Coordinate(red.x(), red.y());
    assertTrue(
        new FieldModel(Alliance.RED).hivePivot().distance(here)
            < new FieldModel(Alliance.BLUE).hivePivot().distance(here),
        "the nearest HIVE is RED's");
  }

  /** FTC fields are six TILES across, so the TILE seams are size_in / 6 apart. */
  private static final int TILES_PER_SIDE = 6;

  /** Tolerance (inches) for a pose written to three decimals to sit on a TILE seam. */
  private static final double SEAM_TOLERANCE_IN = 0.01;

  /** Both points on one TILE seam (a line parallel to a wall, a whole number of TILES out). */
  private static boolean onTileSeam(Coordinate a, Coordinate b) {
    return onTileSeam(a.x, b.x) || onTileSeam(a.y, b.y);
  }

  /** Both coordinates on the same interior TILE seam along one axis. */
  private static boolean onTileSeam(double u, double v) {
    double tile = config.field.size_in / TILES_PER_SIDE;
    double k = Math.rint(u / tile);
    return k > 0
        && k < TILES_PER_SIDE
        && Math.abs(u - k * tile) < SEAM_TOLERANCE_IN
        && Math.abs(v - k * tile) < SEAM_TOLERANCE_IN;
  }

  @Test
  public void upAndDownStartWithTheBackOnTheWallAndASideOnATileSeam() {
    for (Alliance alliance : Alliance.values()) {
      boolean red = alliance == Alliance.RED;
      Pose up = AutoFixtures.up(alliance).start();
      Pose down = AutoFixtures.down(alliance, true).start();
      // Up starts on CELL A's side: the audience wall for RED, the rear wall for BLUE.
      assertEquals(Optional.of(red ? Wall.AUDIENCE : Wall.REAR), backEdgeWall(up));
      assertTrue(onTileSeam(at(up, -back(), half()), at(up, front(), half())), "Up left side");
      // Down is Up flipped across the field's middle: the other wall, the right side on the seam.
      assertEquals(Optional.of(red ? Wall.REAR : Wall.AUDIENCE), backEdgeWall(down));
      assertTrue(
          onTileSeam(at(down, -back(), -half()), at(down, front(), -half())), "Down right side");
      double size = config.field.size_in;
      assertEquals(up.x(), down.x(), EPS, alliance + " Down x");
      assertEquals(size - up.y(), down.y(), EPS, alliance + " Down y");
      assertEquals(
          0.0,
          AngleUnit.normalizeRadians(down.heading() + up.heading()),
          EPS,
          alliance + " Down faces the other way");
    }
  }

  @Test
  public void upAndDownSweepTheRealChassisClearOfTheWallsAndHive() {
    for (Alliance alliance : Alliance.values()) {
      var field = new FieldModel(alliance);
      var side = new Envelope(field.allianceSide());
      side.expandBy(EPS);
      var sentinel = new Sentinel(field);
      for (var routine :
          List.of(
              AutoFixtures.up(alliance),
              AutoFixtures.down(alliance, true),
              AutoFixtures.down(alliance, false))) {
        routine
            .paths()
            .forEach(
                (name, path) -> {
                  var problem =
                      sentinel.routeProblem(path, config.sentinel.route_sample_step, true);
                  assertTrue(problem.isEmpty(), () -> routine + " " + name + ": " + problem.get());
                  for (int i = 0; i <= 200; i++) {
                    Pose pose = path.get(i / 200.0);
                    Polygon robot = outline(pose);
                    String where = routine + " " + name + " at " + pose;
                    assertTrue(
                        side.contains(robot.getEnvelopeInternal()), where + " inside its side");
                    assertFalse(
                        robot.intersects(GF.toGeometry(field.hiveFrame())),
                        where + " clear of the HIVE frame");
                  }
                });
      }
    }
  }
}
