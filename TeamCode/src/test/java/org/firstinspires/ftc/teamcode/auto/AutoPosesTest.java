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
        // Each step's literal points: the Bezier control points then the end, or just the end of a line.
        Map<String, Pose[]> byStep = new LinkedHashMap<>();
        byStep.put("aimAtSouthCell", new Pose[] {AutoPoses.AIM_FROM_START});
        byStep.put("toGarden", new Pose[] {AutoPoses.TO_GARDEN_C1, AutoPoses.TO_GARDEN_C2, AutoPoses.GARDEN_LANE});
        byStep.put("turnSouthAtGarden", new Pose[] {AutoPoses.TURN_SOUTH_AT_GARDEN});
        byStep.put("faceSouthCellFromGarden", new Pose[] {AutoPoses.AIM_FROM_GARDEN});
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
            java.util.List<Json> list = new java.util.ArrayList<>();
            Json control = seg.opt("control");
            if (control != null) for (int i = 0; i < control.size(); i++) list.add(control.at(i));
            if (seg.get("to").opt("ref") == null) list.add(seg.get("to"));   // a literal end point
            Json[] points = list.toArray(new Json[0]);
            assertEquals(e.getValue().length, points.length, e.getKey() + " point count");
            for (int i = 0; i < points.length; i++) {
                assertEquals(points[i].num("xIn"), e.getValue()[i].x(), TOL, e.getKey() + " point " + i + " x");
                assertEquals(points[i].num("yIn"), e.getValue()[i].y(), TOL, e.getKey() + " point " + i + " y");
            }
        }
    }

    @Test
    void turnAndHeldHeadingsMatchTheZenithDraft() throws IOException {
        // Linear turns: {from, to} exactly as written (the numbers also set which way the robot turns).
        Map<String, double[]> turns = new LinkedHashMap<>();
        turns.put("aimAtSouthCell", new double[] {AutoPoses.FACING_NORTH, AutoPoses.SOUTH_CELL_FROM_START});
        turns.put("turnSouthAtGarden", new double[] {AutoPoses.FACING_WEST_NEGATIVE, AutoPoses.FACING_SOUTH});
        turns.put("faceSouthCellFromGarden", new double[] {AutoPoses.FACING_SOUTH, AutoPoses.SOUTH_CELL_FROM_GARDEN});
        turns.put("turnNorth", new double[] {AutoPoses.FACING_WEST, AutoPoses.FACING_NORTH});
        turns.put("turnToFlowerNorth", new double[] {AutoPoses.FACING_NORTH_CELL, AutoPoses.FACING_NORTH});
        turns.put("turnToScoreNorth", new double[] {AutoPoses.FACING_NORTH, AutoPoses.FACING_NORTH_CELL});
        turns.put("turnToPark", new double[] {AutoPoses.FACING_NORTH_CELL, AutoPoses.TOWARD_PARK});
        // Held (constant) headings.
        Map<String, Double> held = new LinkedHashMap<>();
        held.put("sideToGarden", AutoPoses.FACING_SOUTH);
        held.put("noseIntoGarden", AutoPoses.FACING_SOUTH);
        held.put("backOutOfGarden", AutoPoses.FACING_SOUTH);
        held.put("collectFlowerWest", AutoPoses.FACING_WEST);
        held.put("collectFlowerNorth", AutoPoses.FACING_NORTH);

        Map<String, Json> steps = new LinkedHashMap<>();
        collect(read("autos/two-tip.auto.json").get("steps"), steps);
        for (Map.Entry<String, double[]> e : turns.entrySet()) {
            Json h = steps.get(e.getKey()).get("heading");
            assertEquals("linear", h.str("mode"), e.getKey() + " heading mode");
            assertEquals(h.num("fromRad"), e.getValue()[0], 0.001, e.getKey() + " fromRad");
            assertEquals(h.num("toRad"), e.getValue()[1], 0.001, e.getKey() + " toRad");
        }
        for (Map.Entry<String, Double> e : held.entrySet()) {
            Json h = steps.get(e.getKey()).get("heading");
            assertEquals("constant", h.str("mode"), e.getKey() + " heading mode");
            assertEquals(h.num("headingRad"), e.getValue(), 0.001, e.getKey() + " headingRad");
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
