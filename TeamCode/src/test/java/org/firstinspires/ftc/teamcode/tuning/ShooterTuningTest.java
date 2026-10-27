package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

class ShooterTuningTest {

    /** A made-up flywheel with friction and a sagging battery: the fit must recover kS and max speed. */
    @Test
    void theSweepFitRecoversStaticFrictionAndTopSpeedAcrossBatteryVoltage() {
        double kS = 0.05, maxTps = 2600, vRef = 12.0;
        double[] p = {0.2, 0.4, 0.6, 0.8, 1.0};
        double[] v = {12.9, 12.7, 12.4, 12.1, 11.8};   // the battery sags as the power goes up
        double[] s = new double[p.length];
        for (int i = 0; i < p.length; i++) s[i] = (p[i] * v[i] / vRef - kS) * maxTps;

        ShooterTuning.Feedforward ff = ShooterTuning.fitFeedforward(p, v, s, vRef);
        assertEquals(kS, ff.kS, 1e-9);
        assertEquals(maxTps, ff.maxTicksPerSec, 1e-6);
        assertEquals(1.0, ff.r2, 1e-9);
    }

    @Test
    void aCleanStepIsGoodAndARingingOneIsNot() {
        List<Double> t = new ArrayList<>(), clean = new ArrayList<>(), ringing = new ArrayList<>();
        for (int i = 0; i <= 100; i++) {
            double ts = i * 0.02;
            t.add(ts);
            clean.add(2000 * (1 - Math.exp(-ts / 0.15)));
            ringing.add(2000 * (1 - Math.exp(-ts / 0.3) * Math.cos(12 * ts)));
        }
        ShooterTuning.Step a = ShooterTuning.analyzeStep(t, clean, 2000, 0.02);
        assertTrue(a.good(0.02, 0.03), "clean step: " + a);
        assertEquals(0.15 * Math.log(50), a.riseTimeS, 0.03, "first within 2 % after about 3.9 time constants");

        ShooterTuning.Step b = ShooterTuning.analyzeStep(t, ringing, 2000, 0.02);
        assertFalse(b.good(0.02, 0.03), "ringing step: " + b);
        assertTrue(b.overshoot > 0.03);
    }
}
