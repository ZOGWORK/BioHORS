package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePiecePhysicsConfig;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.KinematicBody;
import org.junit.jupiter.api.Test;

/**
 * Calibration and behaviour of the physics hive (BiobuzzHive): with pollen set gently into the up
 * cell one at a time, the 8th tips it and the first 7 do not (team rule), the swing takes a few
 * seconds, and the tipped cell spills its pollen onto the floor on its own side.
 */
class BiobuzzHiveTest {

    static final double R = 1.4;  // pollen radius, SPEC (field file)

    /** How many pollen it took to tip the RED hive from south-up, or -1 if 12 did not. */
    static int pollenToTip(double comHeight) {
        double saved = BiobuzzHive.COM_HEIGHT_IN;
        BiobuzzHive.COM_HEIGHT_IN = comHeight;
        try {
            GamePieceTracker tracker = tracker(BiobuzzHive.body(true));
            String hive = BiobuzzHive.bodyName(true);
            tracker.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
            run(tracker, 1.0);
            for (int n = 1; n <= 12; n++) {
                dropIntoUpCell(tracker, true, false, n);
                run(tracker, 6.0);
                if (tracker.getBodyAngleRad(hive) > 0.0) return n;
            }
            return -1;
        } finally {
            BiobuzzHive.COM_HEIGHT_IN = saved;
        }
    }

    @Test
    void calibrationSweepIsPrinted() {
        double si = BiobuzzHive.INERTIA, sd = BiobuzzHive.DAMPING_PER_S;
        try {
            for (double inertia : new double[] {20000, 30000, 40000}) {
                for (double damp : new double[] {0.6}) {
                    BiobuzzHive.INERTIA = inertia;
                    BiobuzzHive.DAMPING_PER_S = damp;
                    StringBuilder sb = new StringBuilder(String.format(java.util.Locale.ROOT, "CAL I %.0f damp %.1f:", inertia, damp));
                    for (double d = 6.6; d <= 7.81; d += 0.6) {
                        sb.append(String.format(java.util.Locale.ROOT, " com%.2f->%d", d, pollenToTip(d)));
                    }
                    sb.append(String.format(java.util.Locale.ROOT, " swing@8 %.1fs", swingSeconds()));
                    System.out.println(sb);
                }
            }
        } finally {
            BiobuzzHive.INERTIA = si;
            BiobuzzHive.DAMPING_PER_S = sd;
        }
    }

    /** Seconds from the 8th pollen to the far stop (15 = did not get there). */
    static double swingSeconds() {
        GamePieceTracker tracker = tracker(BiobuzzHive.body(true));
        String hive = BiobuzzHive.bodyName(true);
        tracker.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
        run(tracker, 1.0);
        for (int n = 1; n <= 7; n++) {
            dropIntoUpCell(tracker, true, false, n);
            run(tracker, 6.0);
        }
        dropIntoUpCell(tracker, true, false, 8);
        double t = 0;
        while (tracker.getBodyAngleRad(hive) < BiobuzzHive.TILT_RAD - 1e-6 && t < 15) {
            tracker.advance(0.02);
            t += 0.02;
        }
        return t;
    }

    @Test
    void theEighthPollenTipsItAndSevenDoNot() {
        assertEquals(8, pollenToTip(BiobuzzHive.COM_HEIGHT_IN), "team rule: 8 pollen per tip");
    }

    @Test
    void aTipSwingsInAFewSecondsAndSpillsOnItsOwnSide() {
        GamePieceTracker tracker = tracker(BiobuzzHive.body(true));
        String hive = BiobuzzHive.bodyName(true);
        tracker.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
        run(tracker, 1.0);
        for (int n = 1; n <= 7; n++) {
            dropIntoUpCell(tracker, true, false, n);
            run(tracker, 6.0);
        }
        dropIntoUpCell(tracker, true, false, 8);
        double t = 0;
        while (tracker.getBodyAngleRad(hive) < BiobuzzHive.TILT_RAD - 1e-6 && t < 15) {
            tracker.advance(0.02);
            t += 0.02;
        }
        System.out.printf(java.util.Locale.ROOT, "SWING %.2f s%n", t);
        assertTrue(t > 2.5 && t < 6.0, "the swing should take about 4 s (DSim, APPROX), took " + t);
        run(tracker, 4.0);
        int onSouthFloor = 0;
        for (GamePiece p : tracker.getPieces()) {
            if (p.getFieldZInches() < 4.0 && p.getFieldYInches() < -10.0) onSouthFloor++;
        }
        System.out.println("SPILL south floor " + onSouthFloor);
        assertTrue(onSouthFloor >= 6, "most of the load should spill on the south side, got " + onSouthFloor);
    }

    static GamePieceTracker tracker(KinematicBody hive) {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder().pieceMass("pollen", 1.0).build();
        return new GamePieceTracker(1, FieldWorld.builder().perimeter(72, 72).physics(physics).kinematicBody(hive).build());
    }

    /** Sets a pollen down just inside the up cell's open end, spread across its width. */
    static void dropIntoUpCell(GamePieceTracker tracker, boolean red, boolean north, int n) {
        String hive = BiobuzzHive.bodyName(red);
        double y = (north ? 1 : -1) * (BiobuzzHive.OUTER_FACE_Y_IN - R - 0.5);
        double x = -8.0 + ((n - 1) % 6) * 3.2;
        double[] f = new double[3];
        tracker.getWorld().getKinematicBody(hive).toField(tracker.getBodyAngleRad(hive), x, y, R + 0.3, f);
        tracker.addPieceAt("pollen", f[0], f[1], f[2], R);
    }

    static void run(GamePieceTracker tracker, double s) {
        for (double t = 0; t < s; t += 0.02) tracker.advance(0.02);
    }
}
