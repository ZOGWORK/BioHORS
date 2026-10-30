package org.firstinspires.ftc.teamcode.aim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class AimVelocityFilterTest {

    private static final double MAX = 60.0;

    @Test
    void steadyVelocityPassesThrough() {
        AimVelocityFilter f = new AimVelocityFilter(0.5, MAX);
        double[] v = null;
        for (int i = 0; i < 40; i++) v = f.update(20.0, -10.0);
        assertEquals(20.0, v[0], 1e-6);
        assertEquals(-10.0, v[1], 1e-6);
    }

    @Test
    void aSingleFrameSpikeIsAttenuated() {
        AimVelocityFilter f = new AimVelocityFilter(0.5, MAX);
        for (int i = 0; i < 40; i++) f.update(10.0, 0.0);
        double[] spike = f.update(40.0, 0.0); // one bad loop: +30 in/s
        assertEquals(25.0, spike[0], 1e-6, "alpha 0.5 passes half the step");
        double[] after = f.update(10.0, 0.0);
        assertTrue(Math.abs(after[0] - 10.0) < Math.abs(spike[0] - 10.0), "and it decays back");
    }

    @Test
    void theMagnitudeIsClampedToTheDrivetrainTopSpeed() {
        AimVelocityFilter f = new AimVelocityFilter(1.0, MAX);
        double[] v = f.update(300.0, 400.0);
        assertEquals(MAX, Math.hypot(v[0], v[1]), 1e-9);
        assertEquals(0.6, v[0] / MAX, 1e-9, "direction kept");
    }

    @Test
    void aNonFiniteReadingGivesZeroAndResets() {
        AimVelocityFilter f = new AimVelocityFilter(0.5, MAX);
        for (int i = 0; i < 40; i++) f.update(30.0, 30.0);
        double[] v = f.update(Double.NaN, 5.0);
        assertEquals(0.0, v[0], 0.0);
        assertEquals(0.0, v[1], 0.0);
        v = f.update(10.0, 0.0);
        assertEquals(5.0, v[0], 1e-9, "the next reading starts from zero, not the old 30");
        assertEquals(0.0, f.update(0.0, Double.POSITIVE_INFINITY)[0], 0.0);
    }

    @Test
    void aBadAlphaMeansNoSmoothingAndABadTopSpeedMeansZero() {
        AimVelocityFilter nan = new AimVelocityFilter(Double.NaN, MAX);
        assertEquals(12.0, nan.update(12.0, 0.0)[0], 1e-9);
        AimVelocityFilter zero = new AimVelocityFilter(0.0, MAX);
        assertEquals(12.0, zero.update(12.0, 0.0)[0], 1e-9);
        AimVelocityFilter noMax = new AimVelocityFilter(0.5, 0.0);
        assertEquals(0.0, noMax.update(12.0, 0.0)[0], 0.0);
    }
}
