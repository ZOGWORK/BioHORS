package org.firstinspires.ftc.teamcode.tuning;

import java.util.List;

/**
 * The arithmetic behind the Shooter Tuner OpMode, kept free of hardware so it can be unit-tested.
 *
 * Feedforward model (what ShooterController runs):
 *   power = (kS + ticksPerSec / maxTicksPerSec) * (vRef / vBattery)
 * so a sweep point (power p at battery voltage v, steady speed s) normalised to vRef is
 *   p * v / vRef = kS + s / maxTicksPerSec
 * which is a straight line in s: intercept kS, slope 1 / maxTicksPerSec.
 */
public final class ShooterTuning {

    private ShooterTuning() {
    }

    /** Result of the power sweep. */
    public static final class Feedforward {
        /** Power needed just to keep the flywheel turning, at vRef. */
        public final double kS;
        /** Steady speed at full power at vRef, encoder ticks/s (ShooterController.ffReferenceMaxTicksPerSec). */
        public final double maxTicksPerSec;
        /** Goodness of the line fit, 0..1. Below ~0.98 the sweep was noisy or the flywheel slipped. */
        public final double r2;

        Feedforward(double kS, double maxTicksPerSec, double r2) {
            this.kS = kS;
            this.maxTicksPerSec = maxTicksPerSec;
            this.r2 = r2;
        }
    }

    /**
     * Fits the feedforward from sweep points.
     *
     * @param power   commanded power at each point (0..1)
     * @param volts   battery voltage measured at each point
     * @param speed   steady encoder speed at each point, ticks/s
     * @param vRef    the reference voltage the result is normalised to (12 V nominal)
     */
    public static Feedforward fitFeedforward(double[] power, double[] volts, double[] speed, double vRef) {
        int n = power.length;
        if (n < 2 || volts.length != n || speed.length != n) {
            throw new IllegalArgumentException("need at least two matching sweep points");
        }
        double[] y = new double[n];
        for (int i = 0; i < n; i++) y[i] = power[i] * volts[i] / vRef;
        double[] line = fitLine(speed, y);
        double slope = line[1];
        if (!(slope > 0)) throw new IllegalStateException("the flywheel did not speed up with power");
        return new Feedforward(Math.max(0.0, line[0]), 1.0 / slope, line[2]);
    }

    /** Least-squares line y = a + b x. Returns {a, b, r2}. */
    public static double[] fitLine(double[] x, double[] y) {
        int n = x.length;
        double mx = 0, my = 0;
        for (int i = 0; i < n; i++) {
            mx += x[i];
            my += y[i];
        }
        mx /= n;
        my /= n;
        double sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < n; i++) {
            sxx += (x[i] - mx) * (x[i] - mx);
            sxy += (x[i] - mx) * (y[i] - my);
            syy += (y[i] - my) * (y[i] - my);
        }
        double b = sxx == 0 ? 0 : sxy / sxx;
        double a = my - b * mx;
        double r2 = (sxx == 0 || syy == 0) ? 0 : (sxy * sxy) / (sxx * syy);
        return new double[] {a, b, r2};
    }

    /** What a step to a target speed looked like. */
    public static final class Step {
        /** Seconds from the step until the speed first entered the band; NaN if it never did. */
        public final double riseTimeS;
        /** Largest overshoot above target, as a fraction of target (0.03 = 3 %). */
        public final double overshoot;
        /** Times the speed left the band again after first entering it. */
        public final int bandExits;
        /** Largest |error| in the last 40 % of the window, as a fraction of target. */
        public final double steadyError;

        Step(double riseTimeS, double overshoot, int bandExits, double steadyError) {
            this.riseTimeS = riseTimeS;
            this.overshoot = overshoot;
            this.bandExits = bandExits;
            this.steadyError = steadyError;
        }

        /** Reached target, held it, and did not overshoot or ring. */
        public boolean good(double band, double maxOvershoot) {
            return !Double.isNaN(riseTimeS) && overshoot <= maxOvershoot && bandExits <= 1 && steadyError <= band;
        }

        @Override
        public String toString() {
            return String.format(java.util.Locale.ROOT, "rise %.2fs overshoot %.1f%% exits %d steady %.1f%%",
                    riseTimeS, 100 * overshoot, bandExits, 100 * steadyError);
        }
    }

    /**
     * Analyses samples taken after a step to {@code target}.
     *
     * @param t      seconds since the step, ascending
     * @param speed  measured speed (any unit, same as target)
     * @param band   the "at target" band, as a fraction of target (0.02 = within 2 %)
     */
    public static Step analyzeStep(List<Double> t, List<Double> speed, double target, double band) {
        double tol = band * target;
        double rise = Double.NaN;
        double over = 0;
        int exits = 0;
        boolean inBand = false;
        for (int i = 0; i < t.size(); i++) {
            double err = speed.get(i) - target;
            over = Math.max(over, err / target);
            boolean in = Math.abs(err) <= tol;
            if (in && Double.isNaN(rise)) rise = t.get(i);
            if (!Double.isNaN(rise) && inBand && !in) exits++;
            inBand = in;
        }
        double tEnd = t.isEmpty() ? 0 : t.get(t.size() - 1);
        double steady = 0;
        for (int i = 0; i < t.size(); i++) {
            if (t.get(i) >= 0.6 * tEnd) steady = Math.max(steady, Math.abs(speed.get(i) - target) / target);
        }
        return new Step(rise, Math.max(0, over), exits, steady);
    }
}
