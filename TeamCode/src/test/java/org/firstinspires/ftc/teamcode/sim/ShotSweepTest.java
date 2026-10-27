package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.junit.jupiter.api.Test;

import java.util.Locale;

/**
 * Real launches into the physics hive (BiobuzzHive) from the CAD shot geometry (ShotModel): which
 * flywheel rpm seats a pollen in the up cell from how far away. The robot stands on the hive's
 * centre line south of RED's hive, facing north at the up (south) cell.
 */
class ShotSweepTest {

    /** The cell a single pollen ends up seated in (null if none), fired from y facing north. */
    static String fire(double y, double rpm) {
        GamePieceTracker tracker = BiobuzzHiveTest.tracker(BiobuzzHive.body(true));
        String hive = BiobuzzHive.bodyName(true);
        tracker.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
        double[] l = ShotModel.pollenLaunch(-12.75, y, Math.PI / 2, rpm, 0, 0);
        int id = tracker.addHeldPiece("pollen", BiobuzzHiveTest.R);
        tracker.launch(id, "shot", l[0], l[1], l[2], l[3], l[4], l[5]);
        for (double t = 0; t < 3.0; t += 0.02) tracker.advance(0.02);
        return tracker.getPiece(id).getSeatedCellName();
    }

    @Test
    void sweepIsPrinted() {
        StringBuilder head = new StringBuilder("RPM / y  ");
        for (double y = -30; y >= -70; y -= 5) head.append(String.format(Locale.ROOT, "%5.0f", y));
        System.out.println(head);
        for (int rpm = 1600; rpm <= 6000; rpm += 400) {
            StringBuilder row = new StringBuilder(String.format(Locale.ROOT, "%7d  ", rpm));
            for (double y = -30; y >= -70; y -= 5) {
                row.append(String.format(Locale.ROOT, "%5s", BiobuzzHive.cellName(true, false).equals(fire(y, rpm)) ? "IN" : "."));
            }
            System.out.println(row);
        }
    }

    /** The lowest rpm (100 rpm steps, up to 6000) that seats a pollen from y, or -1. */
    static int minRpm(double y) {
        for (int rpm = 1000; rpm <= 6000; rpm += 100) {
            if (BiobuzzHive.cellName(true, false).equals(fire(y, rpm))) return rpm;
        }
        return -1;
    }

    @Test
    void sensitivityToExitAngleAndGripIsPrinted() {
        double savedPitch = ShotModel.POLLEN_PITCH_DEG, savedGrip = ShotModel.GRIP_EFFICIENCY;
        try {
            for (double grip : new double[] {0.5, 0.8}) {
                ShotModel.GRIP_EFFICIENCY = grip;
                StringBuilder head = new StringBuilder(String.format(Locale.ROOT, "MINRPM grip %.1f  pitch/y", grip));
                for (double y = -30; y >= -70; y -= 5) head.append(String.format(Locale.ROOT, "%6.0f", y));
                System.out.println(head);
                for (double pitch : new double[] {44, 50, 55, 60, 65, 70, 75}) {
                    ShotModel.POLLEN_PITCH_DEG = pitch;
                    StringBuilder row = new StringBuilder(String.format(Locale.ROOT, "MINRPM grip %.1f  %5.0f   ", grip, pitch));
                    for (double y = -30; y >= -70; y -= 5) row.append(String.format(Locale.ROOT, "%6d", minRpm(y)));
                    System.out.println(row);
                }
            }
        } finally {
            ShotModel.POLLEN_PITCH_DEG = savedPitch;
            ShotModel.GRIP_EFFICIENCY = savedGrip;
        }
    }

    /** Runs body with the pollen shot set to pitch and grip, then puts ShotModel back. */
    static void withShot(double pitchDeg, double grip, Runnable body) {
        double savedPitch = ShotModel.POLLEN_PITCH_DEG, savedGrip = ShotModel.GRIP_EFFICIENCY;
        ShotModel.POLLEN_PITCH_DEG = pitchDeg;
        ShotModel.GRIP_EFFICIENCY = grip;
        try {
            body.run();
        } finally {
            ShotModel.POLLEN_PITCH_DEG = savedPitch;
            ShotModel.GRIP_EFFICIENCY = savedGrip;
        }
    }

    /** The farthest south the robot's centre can be, facing north: its back 7.8 in behind (CAD). */
    static final double FARTHEST_Y_IN = -72 + 7.8;

    @Test
    void atTheCadReadingNoSpotTheRobotCanReachScores() {
        withShot(44.0, 0.5, () -> {
            for (double y = -25; y >= FARTHEST_Y_IN; y -= 2.5) {
                assertEquals(-1, minRpm(y), "44 deg / 0.5 grip scored from y " + y);
            }
        });
    }

    @Test
    void underTheWhatIfAReachableSpotScoresNearTheTestTarget() {
        withShot(ZenithAutoMatchTest.WHAT_IF_PITCH_DEG, ZenithAutoMatchTest.WHAT_IF_GRIP, () -> {
            int best = Integer.MAX_VALUE;
            for (double y = -40; y >= FARTHEST_Y_IN; y -= 5) {
                int r = minRpm(y);
                if (r > 0) best = Math.min(best, r);
            }
            assertTrue(best <= 2000, "the what-if shot needs " + best + " rpm at best");
        });
    }

    @Test
    void aBallLeavesAtTheModelledSpeedAndAngle() {
        GamePieceTracker tracker = BiobuzzHiveTest.tracker(BiobuzzHive.body(true));
        double[] l = ShotModel.pollenLaunch(-40, -60, Math.PI / 2, 3000, 0, 0);
        int id = tracker.addHeldPiece("pollen", BiobuzzHiveTest.R);
        tracker.launch(id, "shot", l[0], l[1], l[2], l[3], l[4], l[5]);
        tracker.advance(0.02);
        GamePiece p = tracker.getPiece(id);
        double v = Math.hypot(p.getVelocityYInchesPerSecond(), p.getVelocityZInchesPerSecond());
        assertEquals(ShotModel.exitSpeed(3000), v, 0.03 * v);
        assertEquals(-40 + 2.9, p.getFieldXInches(), 0.5, "the pollen shooter is on the right");
    }
}
