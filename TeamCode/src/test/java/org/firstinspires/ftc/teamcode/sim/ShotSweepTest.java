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

    @Test
    void theTestTargetCannotReachTheCellFromAnywhereOnTheCentreLine() {
        for (double y = -25; y >= -70; y -= 2.5) {
            assertEquals(null, fire(y, 1600), "at 1600 rpm the ball tops out about 9 in above the exit; y " + y);
        }
    }

    @Test
    void someReachableRpmAndDistanceScores() {
        boolean any = false;
        for (int rpm = 2000; rpm <= 6000 && !any; rpm += 100) {
            for (double y = -30; y >= -66 && !any; y -= 1) {
                any = BiobuzzHive.cellName(true, false).equals(fire(y, rpm));
            }
        }
        assertTrue(any, "no rpm up to the motor's 6000 free speed seats a pollen from the centre line");
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
