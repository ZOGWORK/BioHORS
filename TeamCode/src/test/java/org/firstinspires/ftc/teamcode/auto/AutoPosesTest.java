package org.firstinspires.ftc.teamcode.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * The hand-written state machine follows the route drafted in Zenith: every point in
 * {@link AutoPoses} equals the one in autos/waypoints.json or autos/two-tip.auto.json. Change the
 * draft first (and re-validate it in Zenith), then copy the number into AutoPoses.
 */
class AutoPosesTest {

    static final double TOL = 0.005;

    @Test
    void namedWaypointsMatchTheZenithDraft() throws IOException {
        Json w = read("autos/waypoints.json").get("waypoints");
        Map<String, Pose> named = new LinkedHashMap<>();
        named.put("start", AutoPoses.START);
        named.put("scoreSouth", AutoPoses.SCORE_SOUTH);
        named.put("gardenApproach", AutoPoses.GARDEN_APPROACH);
        named.put("gardenPickup", AutoPoses.GARDEN_PICKUP);
        named.put("gardenTurn", AutoPoses.GARDEN_TURN);
        named.put("flowerWestApproach", AutoPoses.FLOWER_WEST_APPROACH);
        named.put("flowerWestPickup", AutoPoses.FLOWER_WEST_PICKUP);
        named.put("scoreNorth", AutoPoses.SCORE_NORTH);
        named.put("flowerNorthPickup", AutoPoses.FLOWER_NORTH_PICKUP);
        named.put("park", AutoPoses.PARK);
        for (Map.Entry<String, Pose> e : named.entrySet()) {
            Json j = w.get(e.getKey());
            assertNotNull(j, "the draft has no waypoint " + e.getKey());
            assertEquals(j.num("xIn"), e.getValue().x(), TOL, e.getKey() + " x");
            assertEquals(j.num("yIn"), e.getValue().y(), TOL, e.getKey() + " y");
            assertEquals(0.0, angleBetween(j.num("headingRad"), e.getValue().heading()), 0.001, e.getKey() + " heading");
        }
    }

    @Test
    void pathPointsMatchTheZenithDraft() throws IOException {
        Map<String, Pose[]> byStep = new LinkedHashMap<>();
        byStep.put("faceSouthCell", new Pose[] {AutoPoses.FACE_SOUTH_CELL});
        byStep.put("turnToGarden", new Pose[] {AutoPoses.TURN_TO_GARDEN});
        byStep.put("toGarden", new Pose[] {AutoPoses.GARDEN_LANE});
        byStep.put("turnSouthAtGarden", new Pose[] {AutoPoses.TURN_SOUTH_AT_GARDEN});
        byStep.put("turnToScoreSouth", new Pose[] {AutoPoses.TURN_TO_SCORE_SOUTH});
        byStep.put("toScoreSouth", new Pose[] {AutoPoses.TO_SCORE_SOUTH_C1, AutoPoses.TO_SCORE_SOUTH_C2});
        byStep.put("faceSouthCellAgain", new Pose[] {AutoPoses.FACE_SOUTH_CELL});
        byStep.put("turnToFlowerWest", new Pose[] {AutoPoses.TURN_TO_FLOWER_WEST});
        byStep.put("toFlowerWest", new Pose[] {AutoPoses.TO_FLOWER_WEST_C1, AutoPoses.TO_FLOWER_WEST_C2});
        byStep.put("turnNorth", new Pose[] {AutoPoses.TURN_NORTH});
        byStep.put("aroundWestToScoreNorth", new Pose[] {AutoPoses.AROUND_WEST_C1, AutoPoses.AROUND_WEST_C2});
        byStep.put("turnToFlowerNorth", new Pose[] {AutoPoses.NORTH_OF_SCORE_NORTH});
        byStep.put("backOffFlowerNorth", new Pose[] {AutoPoses.NORTH_OF_SCORE_NORTH});
        byStep.put("turnToPark", new Pose[] {AutoPoses.TURN_TO_PARK});

        Map<String, Json> steps = new LinkedHashMap<>();
        collect(read("autos/two-tip.auto.json").get("steps"), steps);
        for (Map.Entry<String, Pose[]> e : byStep.entrySet()) {
            Json step = steps.get(e.getKey());
            assertNotNull(step, "the draft has no step " + e.getKey());
            Json seg = step.get("segments").at(0);
            Json[] points;
            Json control = seg.opt("control");
            if (control != null) {
                points = new Json[control.size()];
                for (int i = 0; i < points.length; i++) points[i] = control.at(i);
            } else {
                points = new Json[] {seg.get("to")};
            }
            assertEquals(e.getValue().length, points.length, e.getKey() + " point count");
            for (int i = 0; i < points.length; i++) {
                assertEquals(points[i].num("xIn"), e.getValue()[i].x(), TOL, e.getKey() + " point " + i + " x");
                assertEquals(points[i].num("yIn"), e.getValue()[i].y(), TOL, e.getKey() + " point " + i + " y");
            }
        }
    }

    @Test
    void blueIsTheRedRoutePointMirrored() {
        Pose b = AutoPoses.of(Alliance.BLUE, AutoPoses.SCORE_NORTH);
        assertEquals(-AutoPoses.SCORE_NORTH.x(), b.x(), 1e-9);
        assertEquals(-AutoPoses.SCORE_NORTH.y(), b.y(), 1e-9);
        assertEquals(0.0, angleBetween(AutoPoses.SCORE_NORTH.heading() + Math.PI, b.heading()), 1e-9);
        assertEquals(AutoPoses.FACING_SOUTH + Math.PI, AutoPoses.heading(Alliance.BLUE, AutoPoses.FACING_SOUTH), 1e-9);
    }

    /** Pedro's Pose keeps headings in [0, 2 pi): compare angles modulo a full turn. */
    static double angleBetween(double a, double b) {
        return Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
    }

    private static void collect(Json steps, Map<String, Json> out) {
        for (Json s : steps.items()) {
            out.put(s.str("id"), s);
            Json children = s.opt("steps");
            if (children != null) collect(children, out);
        }
    }

    private static Json read(String rel) throws IOException {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) dir = dir.getParentFile();
        File f = new File(dir, rel);
        try (InputStream in = new FileInputStream(f)) {
            return Json.parse(in, f.getName());
        }
    }
}
