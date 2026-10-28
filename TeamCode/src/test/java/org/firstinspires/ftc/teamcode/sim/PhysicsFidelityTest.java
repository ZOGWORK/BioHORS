package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.KinematicBody;
import org.horizon36596.simloop.field.OrientedBox;
import org.horizon36596.simloop.field.PossessionState;
import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.Locale;

/**
 * The physics world BiobuzzMatch runs on (BiobuzzPhysicsWorld): every claim checked against
 * SimLoop's own game-piece physics, nothing hand-animated.
 */
class PhysicsFidelityTest {

    static final double R = BiobuzzMatch.POLLEN_RADIUS_IN;
    static final double DT = 0.02;
    static final double OVERLAP_TOL_IN = 0.05;   // SET BY HAND: what counts as two solids overlapping

    // ---------------------------------------------------------------- 1. ball against ball

    @Test
    void aRollingBallKnocksAStillOneAwayAndTheyNeverOverlap() throws IOException {
        GamePieceTracker t = tracker(2);
        t.updateRobotPose(-60, 60, 0);   // the robot well out of the way
        int still = t.addLoosePiece("pollen", 0, -45, R);
        int rolling = t.addHeldPiece("pollen", R);
        t.launch(rolling, "roll", -15, -45, R, 60, 0, 0);   // along the floor at 60 in/s, straight at it
        double closest = Double.MAX_VALUE;
        for (double s = 0; s < 2.0; s += DT) {
            t.advance(DT);
            closest = Math.min(closest, distance(t.getPiece(still), t.getPiece(rolling)));
        }
        GamePiece a = t.getPiece(still), b = t.getPiece(rolling);
        System.out.printf(Locale.ROOT, "BALLS still one moved to (%.1f, %.1f); closest %.2f in%n",
                a.getFieldXInches(), a.getFieldYInches(), closest);
        assertTrue(a.getFieldXInches() > 3.0, "the struck ball should roll away, it is at x " + a.getFieldXInches());
        assertTrue(closest >= 2 * R - OVERLAP_TOL_IN, "the balls overlapped: centres " + closest + " in apart");
        assertTrue(b.getFieldXInches() < a.getFieldXInches(), "the striker should stay behind the ball it hit");
    }

    @Test
    void pollenInACellPileUpWithoutOverlapping() throws IOException {
        GamePieceTracker t = tracker(8);
        String hive = BiobuzzHive.bodyName(true);
        t.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
        t.setBodyMotion(BiobuzzHive.bodyName(false), BiobuzzHive.angleWithUp(true), 0.0);
        for (int n = 1; n <= 6; n++) {
            BiobuzzHiveTest.dropIntoUpCell(t, true, false, 1);   // all at the same spot: they must pile
            BiobuzzHiveTest.run(t, 0.6);
        }
        BiobuzzHiveTest.run(t, 2.0);
        List<GamePiece> ps = t.getPieces();
        double closest = Double.MAX_VALUE;
        for (int i = 0; i < ps.size(); i++) {
            for (int j = i + 1; j < ps.size(); j++) closest = Math.min(closest, distance(ps.get(i), ps.get(j)));
        }
        System.out.printf(Locale.ROOT, "CELL 6 pollen dropped on one spot: closest centres %.2f in%n", closest);
        assertTrue(closest >= 2 * R - OVERLAP_TOL_IN, "pollen in the cell overlap: centres " + closest + " in apart");
    }

    // ---------------------------------------------------------------- 2. the robot pushes

    @Test
    void theRobotPushesALooseBallItDrivesIntoAndNeverOverlapsIt() throws IOException {
        GamePieceTracker t = tracker(1);
        Json robot = readJson("robot.json");
        t.setRobotShape(BiobuzzPhysicsWorld.robotShape(robot));
        int ball = t.addLoosePiece("pollen", -30, -40, R);
        // Facing +y, driving +y at 30 in/s, the ball 4 in to the left of centre: the left side plate /
        // chassis corner meets it (the mouth is 13.2 in wide, so this is beside it, intake off anyway).
        double x = -30 + 8.0, y = -62;
        double worst = Double.MAX_VALUE;
        for (double s = 0; s < 1.6; s += DT) {
            y += 30 * DT;
            t.updateRobotPose(x, y, Math.PI / 2);
            t.advance(DT);
            GamePiece p = t.getPiece(ball);
            for (OrientedBox part : t.getRobotShape().placedOnRobot(x, y, Math.PI / 2)) {
                double[] c = part.closestPoint(p.getFieldXInches(), p.getFieldYInches(), p.getFieldZInches());
                double gap = Math.sqrt(Math.pow(c[0] - p.getFieldXInches(), 2) + Math.pow(c[1] - p.getFieldYInches(), 2)
                        + Math.pow(c[2] - p.getFieldZInches(), 2));
                worst = Math.min(worst, gap);
            }
        }
        GamePiece p = t.getPiece(ball);
        double moved = Math.hypot(p.getFieldXInches() + 30, p.getFieldYInches() + 40);
        System.out.printf(Locale.ROOT, "PUSH ball moved %.1f in to (%.1f, %.1f); closest to the robot %.2f in%n",
                moved, p.getFieldXInches(), p.getFieldYInches(), worst);
        assertEquals(PossessionState.LOOSE, p.getState(), "intake off: the ball is pushed, not taken");
        assertTrue(moved > 4.0, "the robot should push the ball away, it moved " + moved + " in");
        assertTrue(worst >= R - OVERLAP_TOL_IN, "the ball got inside the robot: " + worst + " in from a part");
    }

    // ---------------------------------------------------------------- 3. the hive's structure

    @Test
    void aShotIntoAFrameLegBouncesOff() throws IOException {
        GamePieceTracker t = tracker(1);
        t.updateRobotPose(-60, 60, 0);
        Json leg = null;
        for (Json o : readJson("biobuzz.field.json").get("obstacles").items()) {
            if (o.str("id").equals("frameLegSouthWest")) leg = o;
        }
        Json a = leg.get("fromIn"), b = leg.get("toIn");
        double lx = a.num("xIn") + 0.5 * (b.num("xIn") - a.num("xIn"));
        double ly = a.num("yIn") + 0.5 * (b.num("yIn") - a.num("yIn"));
        double lz = a.num("zIn") + 0.5 * (b.num("zIn") - a.num("zIn"));
        // Fired at the leg's middle from 14 in to its west, level, at 120 in/s (gravity drop over
        // 0.12 s is under an inch).
        int ball = t.addHeldPiece("pollen", R);
        t.launch(ball, "shot", lx - 14, ly, lz + 0.5, 120, 0, 0);
        double furthestEast = -Double.MAX_VALUE;
        for (double s = 0; s < 0.6; s += DT) {
            t.advance(DT);
            furthestEast = Math.max(furthestEast, t.getPiece(ball).getFieldXInches());
        }
        GamePiece p = t.getPiece(ball);
        System.out.printf(Locale.ROOT, "LEG at x %.1f: ball got to x %.1f, now (%.1f, %.1f, %.1f)%n",
                lx, furthestEast, p.getFieldXInches(), p.getFieldYInches(), p.getFieldZInches());
        assertTrue(furthestEast < lx, "the ball went through the leg (reached x " + furthestEast + ", leg at " + lx + ")");
        assertTrue(p.getFieldXInches() < lx - 1.0, "the ball should bounce back west, it is at x " + p.getFieldXInches());
    }

    @Test
    /**
     * Shots through the up cell's opening seat; shots that clip its floor lip or a side wall's edge
     * glance off and come back out toward the shooter. (A ball that gets fully inside stays: at the
     * hive's restitution, BiobuzzHive.RESTITUTION 0.15, APPROX, NEEDS MEASUREMENT, nothing rattles
     * back out from deep in the cell.)
     */
    void shotsThroughTheOpeningSeatAndShotsThatClipItsEdgeBounceOff() throws IOException {
        int seated = 0, rejected = 0;
        double g = BiobuzzPhysicsWorld.physics().getGravityInchesPerS2();
        for (int k = 0; k < 24; k++) {
            GamePieceTracker t = tracker(1);
            t.updateRobotPose(-60, 60, 0);
            String hive = BiobuzzHive.bodyName(true);
            t.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
            t.setBodyMotion(BiobuzzHive.bodyName(false), BiobuzzHive.angleWithUp(true), 0.0);
            KinematicBody body = t.getWorld().getKinematicBody(hive);
            // Each shot is solved to pass through the middle of the up cell's opening after T
            // seconds, from a fast flat one (short T) to a slow lob (long T): the same target, a
            // different angle and speed coming in.
            double[] target = new double[3];
            // Aim points across the opening, from below its floor lip to above its roof, and from
            // its middle out to a side wall: the edges are where a ball glances in and out.
            double aimZ = -1.5 + (k % 8) * (BiobuzzHive.CELL_HEIGHT_IN + 3.0) / 7.0;
            double aimX = (k / 8) * 4.5;
            body.toField(BiobuzzHive.angleWithUp(false), aimX, -BiobuzzHive.OUTER_FACE_Y_IN + 0.5, aimZ, target);
            double tf = 0.55, x0 = -12.75, y0 = -50, z0 = 16;
            double vx = (target[0] - x0) / tf, vy = (target[1] - y0) / tf, vz = (target[2] - z0 + 0.5 * g * tf * tf) / tf;
            double speed = Math.hypot(vy, vz), pitch = Math.atan2(vz, vy);
            int ball = t.addHeldPiece("pollen", R);
            t.launch(ball, "shot", x0, y0, z0, vx, vy, vz);
            boolean entered = false;
            double[] local = new double[3];
            for (double s = 0; s < 3.0; s += DT) {
                t.advance(DT);
                GamePiece p = t.getPiece(ball);
                body.toLocal(t.getBodyAngleRad(hive), p.getFieldXInches(), p.getFieldYInches(), p.getFieldZInches(), local);
                if (Math.abs(local[0]) < BiobuzzHive.HALF_WIDTH_X_IN && local[1] < -BiobuzzHive.INNER_WALL_Y_IN
                        && local[1] > -BiobuzzHive.OUTER_FACE_Y_IN && local[2] > 0 && local[2] < BiobuzzHive.CELL_HEIGHT_IN) {
                    entered = true;
                }
            }
            GamePiece p = t.getPiece(ball);
            boolean in = p.getSeatedCellName() != null;
            if (entered && in) seated++;
            // Off the lip or a wall edge: never got inside, back on the floor on the shooter's side.
            if (!entered && !in && p.getFieldZInches() < 3.0 && p.getFieldYInches() < -BiobuzzHive.OUTER_FACE_Y_IN) rejected++;
            System.out.printf(Locale.ROOT, "CELL shot %d: %.0f in/s at %.1f deg, entered %s, %s at (%.1f, %.1f, %.1f)%n",
                    k, speed, Math.toDegrees(pitch), entered, in ? "seated" : "loose", p.getFieldXInches(),
                    p.getFieldYInches(), p.getFieldZInches());
        }
        assertTrue(seated > 0, "no shot stayed in the cell");
        assertTrue(rejected > 0, "no shot glanced off the cell's lip or wall edge back toward the shooter");
    }

    // ---------------------------------------------------------------- 4. both hives

    @Test
    void bothHivesTipUnderTheirLoad() throws IOException {
        GamePieceTracker t = tracker(16);
        String redHive = BiobuzzHive.bodyName(true), blueHive = BiobuzzHive.bodyName(false);
        t.setBodyMotion(redHive, BiobuzzHive.angleWithUp(false), 0.0);   // RED south up at kickoff
        t.setBodyMotion(blueHive, BiobuzzHive.angleWithUp(true), 0.0);   // BLUE north up
        BiobuzzHiveTest.run(t, 1.0);
        for (int n = 1; n <= 8; n++) {
            BiobuzzHiveTest.dropIntoUpCell(t, true, false, n);
            BiobuzzHiveTest.dropIntoUpCell(t, false, true, n);
            BiobuzzHiveTest.run(t, 3.0);
        }
        BiobuzzHiveTest.run(t, 6.0);
        double red = Math.toDegrees(t.getBodyAngleRad(redHive)), blue = Math.toDegrees(t.getBodyAngleRad(blueHive));
        System.out.printf(Locale.ROOT, "BOTH HIVES after 8 each: red %.1f deg, blue %.1f deg%n", red, blue);
        assertEquals(30.0, red, 1.0, "RED's hive should tip to its north stop");
        assertEquals(-30.0, blue, 1.0, "BLUE's hive should tip to its south stop");
    }

    // ---------------------------------------------------------------- helpers

    static GamePieceTracker tracker(int capacity) throws IOException {
        return new GamePieceTracker(capacity, BiobuzzPhysicsWorld.world(readJson("biobuzz.field.json")));
    }

    static double distance(GamePiece a, GamePiece b) {
        return Math.sqrt(Math.pow(a.getFieldXInches() - b.getFieldXInches(), 2)
                + Math.pow(a.getFieldYInches() - b.getFieldYInches(), 2)
                + Math.pow(a.getFieldZInches() - b.getFieldZInches(), 2));
    }

    static Json readJson(String name) throws IOException {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) dir = dir.getParentFile();
        File f = new File(dir, "TeamCode/src/main/assets/autos/" + name);
        try (InputStream in = new FileInputStream(f)) {
            return Json.parse(in, f.getName());
        }
    }
}
