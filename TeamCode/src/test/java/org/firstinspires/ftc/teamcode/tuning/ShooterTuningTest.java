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

    // ---------------------------------------------------------------- the P search

    private static final double BAND = 0.02, OVER = 0.03;
    private static final ShooterTuning.Step WEAK = new ShooterTuning.Step(Double.NaN, 0, 0, 0.05);
    private static final ShooterTuning.Step RINGING = new ShooterTuning.Step(0.1, 0.06, 3, 0.01);

    private static ShooterTuning.Step good(double rise) {
        return new ShooterTuning.Step(rise, 0.01, 0, 0.005);
    }

    private static ShooterTuning.PSearch search() {
        return new ShooterTuning.PSearch(0.0002, 1.6, 0.00001, 0.05, BAND, OVER);
    }

    @Test
    void theSearchKeepsTheFastestGoodPAndStopsWhenItOvershoots() {
        ShooterTuning.PSearch s = search();
        s.report(WEAK);                       // 0.0002: not there yet
        s.report(good(0.30));                 // 0.00032
        double fastest = s.current();
        s.report(good(0.20));                 // 0.000512: fastest
        s.report(RINGING);                    // 0.0008192: too far
        assertTrue(s.done());
        assertFalse(s.failed());
        assertEquals(fastest, s.best(), 1e-12);
    }

    @Test
    void ifTheFirstStepAlreadyOvershootsItSearchesDownward() {
        ShooterTuning.PSearch s = search();
        s.report(RINGING);                    // 0.0002 too aggressive
        assertTrue(s.current() < 0.0002, "next P should be lower, was " + s.current());
        s.report(RINGING);                    // 0.000125 still too aggressive
        double low = s.current();             // 0.000078
        s.report(good(0.4));
        assertTrue(s.done());
        assertEquals(low, s.best(), 1e-12);
    }

    @Test
    void equalRiseTimesKeepTheFirstGainNotTheLargest() {
        ShooterTuning.PSearch s = search();
        double first = s.current();
        s.report(good(0.2));
        s.report(good(0.2));                  // no faster
        s.report(good(0.2));                  // no faster again: stop
        assertTrue(s.done());
        assertEquals(first, s.best(), 1e-12, "the gain paired with the best rise, not the last one tried");
    }

    @Test
    void noGoodStepIsAFailureNotAZeroGain() {
        ShooterTuning.PSearch up = search();
        while (!up.done()) up.report(WEAK);
        assertTrue(up.failed(), "never reached target");

        ShooterTuning.PSearch down = search();
        down.report(RINGING);
        down.report(WEAK);                    // too weak right below too aggressive
        assertTrue(down.failed());
    }

    @Test
    void maxErrorSeesADipThatRecoversBeforeTheEnd() {
        List<Double> rpm = new ArrayList<>();
        for (int i = 0; i < 100; i++) rpm.add(i >= 10 && i < 20 ? 1900.0 : 2000.0); // 5 % dip early on
        assertEquals(0.05, ShooterTuning.maxError(rpm, 2000), 1e-12);
        assertEquals(0.0, ShooterTuning.analyzeStep(times(100), rpm, 2000, 0.02).steadyError, 1e-12,
                "the step analysis only looks at the end, which is why the together check uses maxError");
    }

    private static List<Double> times(int n) {
        List<Double> t = new ArrayList<>();
        for (int i = 0; i < n; i++) t.add(i * 0.02);
        return t;
    }
}
