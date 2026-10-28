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
     * The piecewise headings the state machine builds its paths from (AutoPoses.*_HEADING): every
     * piece's end t, mode and headings exactly as in the draft (the numbers also set which way the
     * robot turns).
     */
    @Test
    void piecewiseHeadingsMatchTheZenithDraft() throws IOException {
        Map<String, AutoPoses.HeadingPiece[]> expected = new LinkedHashMap<>();
        expected.put("toOtherSide", AutoPoses.TO_OTHER_SIDE_HEADING);
        expected.put("toScoreNorth", AutoPoses.TO_SCORE_NORTH_HEADING);
        expected.put("toGarden", AutoPoses.TO_GARDEN_HEADING);
        expected.put("driveBack", AutoPoses.DRIVE_BACK_HEADING);
        Map<String, Json> steps = steps();
        for (Map.Entry<String, AutoPoses.HeadingPiece[]> e : expected.entrySet()) {
            Json h = steps.get(e.getKey()).get("heading");
            assertEquals("piecewise", h.str("mode"), e.getKey());
            Json ranges = h.get("ranges");
            assertEquals(e.getValue().length, ranges.size(), e.getKey() + " ranges");
            for (int i = 0; i < ranges.size(); i++) {
                AutoPoses.HeadingPiece want = e.getValue()[i];
                Json r = ranges.at(i);
                Json rh = r.get("heading");
                String at = e.getKey() + " range " + i;
                assertEquals(want.endT, r.num("endT"), 1e-4, at + " endT");
                assertEquals(want.mode, rh.str("mode"), at + " mode");
                if (want.mode.equals("linear")) {
                    assertEquals(want.from, rh.num("fromRad"), 1e-4, at + " fromRad");
                    assertEquals(want.to, rh.num("toRad"), 1e-4, at + " toRad");
                } else if (want.mode.equals("constant")) {
                    assertEquals(want.from, rh.num("headingRad"), 1e-4, at + " headingRad");
                }
            }
        }
    }

    /**
     * No heading jump anywhere on a curve, and no tangent piece (Pedro misreads one inside a
     * piecewise heading): each piece starts on the heading the one before it ends
     * on (or, for the first, the heading the robot has where the leg starts), and the last ends on
     * the heading the next step holds. A jump is the robot spinning round on the spot mid-path.
     */
    @Test
    void curveHeadingsNeverJump() {
        check("toOtherSide", AutoPoses.SCORE_SOUTH.heading(), AutoPoses.TO_OTHER_SIDE_HEADING, AutoPoses.FACING_NORTH);
        check("toScoreNorth", AutoPoses.FLOWER_NORTH_PICKUP.heading(), AutoPoses.TO_SCORE_NORTH_HEADING,
                AutoPoses.SCORE_NORTH.heading());
        check("toGarden", AutoPoses.SCORE_NORTH.heading(), AutoPoses.TO_GARDEN_HEADING, AutoPoses.FACING_WEST);
        check("driveBack", AutoPoses.OFF_GARDEN_LANE.heading(), AutoPoses.DRIVE_BACK_HEADING,
                AutoPoses.SCORE_NORTH.heading());
    }

    private static void check(String leg, double startHeading, AutoPoses.HeadingPiece[] pieces, double endHeading) {
        double before = startHeading;
        double startT = 0;
        for (int i = 0; i < pieces.length; i++) {
            AutoPoses.HeadingPiece p = pieces[i];
            assertEquals(0.0, Math.toDegrees(angleBetween(before, pieceAt(p, true))), 2.0,
                    leg + " jumps at t = " + startT + " (piece " + i + ")");
            before = pieceAt(p, false);
            startT = p.endT;
        }
        assertEquals(0.0, Math.toDegrees(angleBetween(before, endHeading)), 2.0, leg + " ends off the next heading");
    }

    /** A piece's heading where it starts ({@code start}) or ends. */
    private static double pieceAt(AutoPoses.HeadingPiece p, boolean start) {
        switch (p.mode) {
            case "linear": return start ? p.from : p.to;
            case "constant": return p.from;
            default:
                // Pedro 3's piecewise heading hands each piece its own 0..1 t, so a tangent piece
                // reads the direction of the wrong part of the curve: write it as linear pieces
                // (tools/zenith_headings.py).
                throw new AssertionError("a " + p.mode + " piece inside a piecewise heading: Pedro misreads it");
        }
    }

    @Test
    void heldHeadingsMatchTheZenithDraft() throws IOException {
        Map<String, Double> held = new LinkedHashMap<>();
        held.put("collectFlower", AutoPoses.FACING_NORTH);
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
