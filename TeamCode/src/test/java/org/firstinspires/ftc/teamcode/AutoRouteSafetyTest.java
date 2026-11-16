package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Checks every configured routine and emergency parking from positions along each route.
 *
 * <p>This is the only route check: the robot no longer re-checks routes at INIT. It runs with every
 * APK build and, tagged {@code push-config}, in {@code checkPushConfig} before {@code pushConfig}
 * sends a config to the robot.
 *
 * <p>The route and turn checks use Sentinel's footprint and are skipped while {@code
 * sentinel.footprint_safety_enabled} is false. That square is centred on the pose, which is the
 * spin centre 6.75 in from the back edge, so it crosses the wall behind every start pose; the start
 * poses themselves are checked against the real outline in {@link StartPoseGeometryTest}.
 */
@Tag("push-config")
public class AutoRouteSafetyTest {
  private static void assumeFootprintSafety() {
    assumeTrue(
        config.sentinel.footprint_safety_enabled,
        "sentinel.footprint_safety_enabled is false: footprint route checks are off");
  }

  @Test
  public void parksEarnLeaveAndPark() {
    for (Alliance alliance : Alliance.values()) {
      Sentinel sentinel = new Sentinel(alliance);
      for (var routine : AutoFixtures.routines(alliance)) {
        if (routine.parks()) {
          assertTrue(
              sentinel.earnsLeaveAndPark(routine.park()), routine + " park earns LEAVE and PARK");
        } else {
          assertFalse(sentinel.isTouchingWall(routine.park()), routine + " end earns LEAVE");
          assertFalse(sentinel.isParked(routine.park()), routine + " end stays out of the zone");
        }
      }
    }
  }

  /**
   * Every park pose, AUTO's and TeleOp's, has exactly one wheel in the LOADING ZONE: that wheel at
   * least {@code sentinel.park_wheel_margin_in} inside, the other three at least that far outside.
   */
  @Test
  public void everyParkPoseHasExactlyOneWheelInTheLoadingZone() {
    double margin = config.sentinel.park_wheel_margin_in;
    for (Alliance alliance : Alliance.values()) {
      Sentinel sentinel = new Sentinel(alliance);
      Map<String, Pose> parks = new LinkedHashMap<>();
      for (var routine : AutoFixtures.routines(alliance)) {
        if (routine.parks()) {
          parks.put(routine + " park", routine.park());
        }
      }
      parks.put(alliance + " TeleOp park", MatchProfile.loadMatchProfile(alliance).parkPose());
      parks.forEach(
          (name, park) -> {
            double[] depths = sentinel.wheelDepthsInLoadingZone(park);
            long in = Arrays.stream(depths).filter(d -> d >= margin - 1e-6).count();
            long out = Arrays.stream(depths).filter(d -> d <= -margin + 1e-6).count();
            String detail = name + " wheel depths " + Arrays.toString(depths);
            assertEquals(1, in, detail + ": one wheel in");
            assertEquals(3, out, detail + ": three wheels out");
          });
    }
  }

  @Test
  public void everyConfiguredPathStaysClearAndOnItsAllianceSide() {
    assumeFootprintSafety();
    for (Alliance alliance : Alliance.values()) {
      Sentinel sentinel = new Sentinel(alliance);
      for (var routine : AutoFixtures.routines(alliance)) {
        routine.paths().forEach((name, path) -> assertSafe(sentinel, routine + " " + name, path));
      }
    }
  }

  @Test
  public void parkGuardCanSafelyExitEveryConfiguredRoute() {
    assumeFootprintSafety();
    for (Alliance alliance : Alliance.values()) {
      Sentinel sentinel = new Sentinel(alliance);
      for (var routine : AutoFixtures.routines(alliance)) {
        routine
            .paths()
            .forEach(
                (name, path) -> {
                  for (int i = 0; i <= 20; i++) {
                    Pose from = path.get(i / 20.0);
                    var route = sentinel.parkRoute(from, routine.park());
                    assertFalse(
                        route.isEmpty(), routine + " " + name + " has a park route at " + i);
                    for (Pose to : route) {
                      if (from.distance(to) > 0.0) {
                        assertSafe(
                            sentinel, routine + " " + name + " park at " + i, pline(from, to));
                      }
                      from = to;
                    }
                  }
                });
      }
    }
  }

  /**
   * Every volley pose must point the robot close enough at its CELL that the turret can cover the
   * rest (and a turret-less robot only turns that far in place): a pose that looks fine on the
   * field map can still leave the target outside the turret's travel.
   */
  @Test
  public void everyVolleyPoseFacesItsCellWithinTurretReach() {
    double reachDeg = Math.min(-config.turret.travel.min_angle, config.turret.travel.max_angle);
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      for (AutoFixtures.Volley v : AutoFixtures.allVolleys(alliance)) {
        FieldModel.Opening o = field.upOpening(v.cell());
        double bearing = Math.atan2(o.y() - v.pose().y(), o.x() - v.pose().x());
        double errorDeg = Math.toDegrees(AngleUnit.normalizeRadians(bearing - v.pose().heading()));
        assertTrue(
            Math.abs(errorDeg) <= reachDeg,
            alliance + " " + v.name() + " is " + errorDeg + " deg off its CELL");
      }
    }
  }

  /**
   * With the turret off ({@code turret.enabled: false}) the chassis turns in place to face the CELL
   * before each volley ({@code ShotController.shotHoldPose}); a turn that would sweep into the HIVE
   * frame is refused and that volley never fires. Every volley pose must be able to make that turn
   * inside the field and, in AUTO, on the alliance side (G402).
   */
  @Test
  public void everyVolleyPoseCanTurnInPlaceToFaceItsCell() {
    assumeFootprintSafety();
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      Sentinel sentinel = new Sentinel(field);
      for (var routine : AutoFixtures.routines(alliance)) {
        for (AutoFixtures.Volley v : routine.volleys()) {
          assertTurnToCellClear(sentinel, field, routine + " " + v.name(), v, true);
        }
      }
      for (AutoFixtures.Volley v : AutoFixtures.teleopVolleys(alliance)) {
        assertTurnToCellClear(sentinel, field, alliance + " " + v.name(), v, false);
      }
    }
  }

  private static void assertTurnToCellClear(
      Sentinel sentinel,
      FieldModel field,
      String name,
      AutoFixtures.Volley v,
      boolean requireAllianceSide) {
    FieldModel.Opening o = field.upOpening(v.cell());
    Pose pivot = Turret.pivotPose(v.pose());
    double bearing = Math.atan2(o.y() - pivot.y(), o.x() - pivot.x());
    var problem = sentinel.rotationProblem(v.pose(), bearing, requireAllianceSide);
    assertTrue(problem.isEmpty(), name + " turning to face its CELL: " + problem.orElse("clear"));
  }

  private static void assertSafe(Sentinel sentinel, String name, Path path) {
    var problem = sentinel.routeProblem(path, 0.005, true);
    assertTrue(problem.isEmpty(), name + ": " + problem.orElse("clear"));
  }
}
