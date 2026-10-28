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

    /** Seconds from the pollen that tips it to the far stop (15 = did not get there). */
    static double swingSeconds() {
        GamePieceTracker tracker = tracker(BiobuzzHive.body(true));
        String hive = BiobuzzHive.bodyName(true);
        tracker.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
        run(tracker, 1.0);
        // Set pollen down one at a time until the next one tips it, then time that swing.
        int tipAt = pollenToTip(BiobuzzHive.COM_HEIGHT_IN);
        for (int n = 1; n < tipAt; n++) {
            dropIntoUpCell(tracker, true, false, n);
            run(tracker, 6.0);
        }
        dropIntoUpCell(tracker, true, false, tipAt);
        double t = 0;
        while (tracker.getBodyAngleRad(hive) < BiobuzzHive.TILT_RAD - 1e-6 && t < 15) {
            tracker.advance(0.02);
            t += 0.02;
        }
        return t;
    }

    /**
     * How many real shots (ShotModel, the what-if exit, AUTO_POLLEN_RPM from 48 in south of RED's hive)
     * it takes to tip it from south-up, one every 0.3 s as the robot feeds; -1 if 12 did not.
     */
    static int shotsToTip(double comHeight) {
        return shotsToTip(comHeight, false);
    }

    /** As shotsToTip, with the 3 nectar staged in the up cell first (BiobuzzPhysicsWorld.stageNectar). */
    static int shotsToTip(double comHeight, boolean nectar) {
        double saved = BiobuzzHive.COM_HEIGHT_IN;
        BiobuzzHive.COM_HEIGHT_IN = comHeight;
        int[] out = {-1};
        try {
            ShotSweepTest.withShot(ZenithAutoMatchTest.WHAT_IF_PITCH_DEG, ZenithAutoMatchTest.WHAT_IF_GRIP, () -> {
                GamePieceTracker tracker = new GamePieceTracker(12, FieldWorld.builder().perimeter(72, 72)
                        .physics(BiobuzzPhysicsWorld.physics()).kinematicBody(BiobuzzHive.body(true)).build());
                String hive = BiobuzzHive.bodyName(true);
                tracker.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
                if (nectar) BiobuzzPhysicsWorld.stageNectar(tracker, true, false);
                run(tracker, nectar ? 3.0 : 1.0);
                for (int n = 1; n <= 12 && out[0] < 0; n++) {
                    double[] l = ShotModel.pollenLaunch(-13.0, -48.0, Math.PI / 2,
                            org.firstinspires.ftc.teamcode.subsystems.MechanismConstants.AUTO_POLLEN_RPM, 0, 0);
                    int id = tracker.addHeldPiece("pollen", R);
                    tracker.launch(id, "shot", l[0], l[1], l[2], l[3], l[4], l[5]);
                    run(tracker, 0.3);
                    // Settle before the next shot counts: a tip shows within a couple of seconds.
                    if (n >= (nectar ? 1 : 6)) {
                        run(tracker, 2.0);
                        if (tracker.getBodyAngleRad(hive) > -BiobuzzHive.TILT_RAD + Math.toRadians(5)) out[0] = n;
                    }
                }
            });
            return out[0];
        } finally {
            BiobuzzHive.COM_HEIGHT_IN = saved;
        }
    }

    @Test
    void shotSweepIsPrinted() {
        double savedI = BiobuzzHive.INERTIA;
        try {
            for (double inertia : new double[] {14000, 18000, 22000}) {
                BiobuzzHive.INERTIA = inertia;
                for (double com = 5.0; com <= 6.61; com += 0.2) {
                    double savedC = BiobuzzHive.COM_HEIGHT_IN;
                    BiobuzzHive.COM_HEIGHT_IN = com;
                    double swing;
                    try {
                        swing = swingSeconds();
                    } finally {
                        BiobuzzHive.COM_HEIGHT_IN = savedC;
                    }
                    System.out.printf(java.util.Locale.ROOT, "SHOTS I %.0f com %.1f: shot %d, swing %.1f s%n",
                            inertia, com, shotsToTip(com), swing);
                }
            }
        } finally {
            BiobuzzHive.INERTIA = savedI;
        }
    }

    @Test
    void theEighthPollenTipsItAndSevenDoNot() {
        assertEquals(8, shotsToTip(BiobuzzHive.COM_HEIGHT_IN), "an empty cell: 8 shot pollen tip it (season table)");
    }

    @Test
    void aTipSwingsInAFewSecondsAndSpillsOnItsOwnSide() {
        GamePieceTracker tracker = tracker(BiobuzzHive.body(true));
        String hive = BiobuzzHive.bodyName(true);
        tracker.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
        run(tracker, 1.0);
        // Set pollen down one at a time until the next one tips it, then time that swing.
        int tipAt = pollenToTip(BiobuzzHive.COM_HEIGHT_IN);
        for (int n = 1; n < tipAt; n++) {
            dropIntoUpCell(tracker, true, false, n);
            run(tracker, 6.0);
        }
        dropIntoUpCell(tracker, true, false, tipAt);
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
        assertTrue(onSouthFloor >= tipAt - 2, "most of the load should spill on the south side, got " + onSouthFloor);
    }

    static GamePieceTracker tracker(KinematicBody hive) {
        // The same physics the match runs on (BiobuzzPhysicsWorld), sub-step included: the hold is
        // calibrated to it.
        return new GamePieceTracker(1, FieldWorld.builder().perimeter(72, 72).physics(BiobuzzPhysicsWorld.physics())
                .kinematicBody(hive).build());
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
