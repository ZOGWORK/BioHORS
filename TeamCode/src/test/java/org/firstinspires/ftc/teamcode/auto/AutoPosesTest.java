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
 * The hand-written state machine follows the route tuned in Zenith: every point in
 * {@link AutoPoses} equals the one in autos/waypoints.json or autos/two-tip.auto.json, and the
 * headings TwoTipStateMachineAuto uses equal the draft's. Change the draft first (and re-validate it
 * in Zenith), then copy the number over.
 */
class AutoPosesTest {

    static final double TOL = 0.005;

    @Test
    void namedWaypointsMatchTheZenithDraft() throws IOException {
        Json w = read("autos/waypoints.json").get("waypoints");
        Map<String, Pose> named = new LinkedHashMap<>();
        named.put("start", AutoPoses.START);
        named.put("scoreSouth", AutoPoses.SCORE_SOUTH);
        named.put("scoreNorth", AutoPoses.SCORE_NORTH);
        named.put("flowerNorthApproach", AutoPoses.FLOWER_NORTH_APPROACH);
        named.put("flowerNorthPickup", AutoPoses.FLOWER_NORTH_PICKUP);
        named.put("aboveGardenLane", AutoPoses.ABOVE_GARDEN_LANE);
        named.put("gardenSweepStart", AutoPoses.GARDEN_SWEEP_START);
        named.put("gardenSweepEnd", AutoPoses.GARDEN_SWEEP_END);
        named.put("offGardenLane", AutoPoses.OFF_GARDEN_LANE);
        for (Map.Entry<String, Pose> e : named.entrySet()) {
            Json j = w.get(e.getKey());
            assertNotNull(j, "the draft has no waypoint " + e.getKey());
            assertEquals(j.num("xIn"), e.getValue().x(), TOL, e.getKey() + " x");
            assertEquals(j.num("yIn"), e.getValue().y(), TOL, e.getKey() + " y");
            assertEquals(0.0, angleBetween(j.num("headingRad"), e.getValue().heading()), 0.001, e.getKey() + " heading");
        }
    }

    @Test
    void controlPointsMatchTheZenithDraft() throws IOException {
        Map<String, Pose[]> byStep = new LinkedHashMap<>();
        byStep.put("toOtherSide", AutoPoses.TO_OTHER_SIDE);
        byStep.put("toScoreNorth", AutoPoses.TO_SCORE_NORTH);
        byStep.put("toGarden", AutoPoses.TO_GARDEN);
        byStep.put("driveBack", AutoPoses.DRIVE_BACK);
        Map<String, Json> steps = steps();
        for (Map.Entry<String, Pose[]> e : byStep.entrySet()) {
            Json step = steps.get(e.getKey());
            assertNotNull(step, "the draft has no step " + e.getKey());
            Json control = step.get("segments").at(0).get("control");
            assertEquals(e.getValue().length, control.size(), e.getKey() + " control point count");
            for (int i = 0; i < control.size(); i++) {
                assertEquals(control.at(i).num("xIn"), e.getValue()[i].x(), TOL, e.getKey() + " point " + i + " x");
                assertEquals(control.at(i).num("yIn"), e.getValue()[i].y(), TOL, e.getKey() + " point " + i + " y");
            }
        }
    }

    /**
     * The piecewise headings TwoTipStateMachineAuto writes out by hand: each range's end t and
     * linear from/to, exactly as in the draft (the numbers also set which way the robot turns).
     * {endT, fromRad, toRad} for linear ranges, {endT} for the others.
     */
    @Test
    void piecewiseHeadingsMatchTheZenithDraft() throws IOException {
        Map<String, double[][]> expected = new LinkedHashMap<>();
        expected.put("toOtherSide", new double[][] {{0.2038, 1.5708, 2.3722}, {0.8679}, {1.0, 1.0385, 1.5708}});
        expected.put("toScoreNorth", new double[][] {{0.2041}, {0.5442, 2.8274, 4.7124}, {1.0}});
        expected.put("toGarden", new double[][] {{0.2186, -1.5708, -2.0354}, {1.0}});
        expected.put("driveBack", new double[][] {{0.1612, -3.1416, -1.7852}, {0.707}, {0.8242, -2.0008, -1.5708}, {1.0}});
        Map<String, Json> steps = steps();
        for (Map.Entry<String, double[][]> e : expected.entrySet()) {
            Json h = steps.get(e.getKey()).get("heading");
            assertEquals("piecewise", h.str("mode"), e.getKey());
            Json ranges = h.get("ranges");
            assertEquals(e.getValue().length, ranges.size(), e.getKey() + " ranges");
            for (int i = 0; i < ranges.size(); i++) {
                double[] want = e.getValue()[i];
                Json r = ranges.at(i);
                assertEquals(want[0], r.num("endT"), 1e-4, e.getKey() + " range " + i + " endT");
                if (want.length == 3) {
                    assertEquals("linear", r.get("heading").str("mode"), e.getKey() + " range " + i);
                    assertEquals(want[1], r.get("heading").num("fromRad"), 1e-4, e.getKey() + " range " + i + " from");
                    assertEquals(want[2], r.get("heading").num("toRad"), 1e-4, e.getKey() + " range " + i + " to");
                }
            }
        }
    }

    @Test
    void heldHeadingsMatchTheZenithDraft() throws IOException {
        Map<String, Double> held = new LinkedHashMap<>();
        held.put("collectFlower", AutoPoses.FACING_NORTH);
        held.put("backOffFlower", AutoPoses.FACING_NORTH);
        held.put("slideOntoLane", AutoPoses.FACING_WEST);
        held.put("sweepGarden", AutoPoses.FACING_WEST);
        held.put("offGardenLane", AutoPoses.FACING_WEST);
        Map<String, Json> steps = steps();
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

    private static Map<String, Json> steps() throws IOException {
        Map<String, Json> steps = new LinkedHashMap<>();
        collect(read("autos/two-tip.auto.json").get("steps"), steps);
        return steps;
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
