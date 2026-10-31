package org.firstinspires.ftc.teamcode.aim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class FireGateTest {

    private static final double TOL = Math.toRadians(2.0);
    private static final double ON_TARGET = Math.toRadians(1.0);

    private static FireGate.Shooter atSpeed(String label) {
        return new FireGate.Shooter(label, true, true, 1600.0, 1590.0);
    }

    private static FireGate.Shooter slow(String label) {
        return new FireGate.Shooter(label, true, false, 1600.0, 1350.0);
    }

    private static FireGate.Shooter off(String label) {
        return new FireGate.Shooter(label, false, false, 1600.0, 0.0);
    }

    @Test
    void clearOnlyWhenPoseOkOnTargetAndEveryOnShooterAtSpeed() {
        FireGate.Decision d = FireGate.decide(true, ON_TARGET, TOL, atSpeed("Pollen"), atSpeed("Nectar"));
        assertTrue(d.clear);
        assertEquals("", d.blockedReason);
    }

    @Test
    void anInvalidPoseAloneBlocksAndSaysSo() {
        FireGate.Decision d = FireGate.decide(false, ON_TARGET, TOL, atSpeed("Pollen"), atSpeed("Nectar"));
        assertFalse(d.clear);
        assertEquals("POSE INVALID", d.blockedReason);
    }

    @Test
    void beingOffTargetAloneBlocksAndGivesTheErrorInDegrees() {
        FireGate.Decision d = FireGate.decide(true, Math.toRadians(-4.3), TOL, atSpeed("Pollen"), atSpeed("Nectar"));
        assertFalse(d.clear);
        assertEquals("AIM -4.3 deg off (tol 2.0)", d.blockedReason);
    }

    @Test
    void exactlyAtTheToleranceIsOnTarget() {
        assertTrue(FireGate.decide(true, TOL, TOL, atSpeed("Pollen")).clear);
        assertTrue(FireGate.decide(true, -TOL, TOL, atSpeed("Pollen")).clear);
    }

    @Test
    void aSlowOnShooterAloneBlocksAndGivesItsRpmError() {
        FireGate.Decision d = FireGate.decide(true, ON_TARGET, TOL, atSpeed("Pollen"), slow("Nectar"));
        assertFalse(d.clear);
        assertEquals("Nectar flywheel -250 rpm", d.blockedReason);
    }

    @Test
    void anOffShooterDoesNotBlockWhileAnotherIsOn() {
        assertTrue(FireGate.decide(true, ON_TARGET, TOL, atSpeed("Pollen"), off("Nectar")).clear);
        assertTrue(FireGate.decide(true, ON_TARGET, TOL, off("Pollen"), atSpeed("Nectar")).clear);
    }

    @Test
    void everyShooterOffBlocks() {
        FireGate.Decision d = FireGate.decide(true, ON_TARGET, TOL, off("Pollen"), off("Nectar"));
        assertFalse(d.clear, "open gates with both flywheels stopped would feed pollen into nothing");
        assertEquals("NO SHOOTER ON", d.blockedReason);
        assertFalse(FireGate.decide(true, ON_TARGET, TOL).clear, "no shooters at all");
    }

    @Test
    void everyFailingConditionIsNamed() {
        FireGate.Decision d = FireGate.decide(true, Math.toRadians(10.0), TOL, slow("Pollen"), slow("Nectar"));
        assertFalse(d.clear);
        assertEquals("AIM 10.0 deg off (tol 2.0); Pollen flywheel -250 rpm; Nectar flywheel -250 rpm",
                d.blockedReason);
    }

    @Test
    void whatItCannotCheckBlocks() {
        assertFalse(FireGate.decide(true, Double.NaN, TOL, atSpeed("Pollen")).clear);
        assertFalse(FireGate.decide(true, ON_TARGET, Double.NaN, atSpeed("Pollen")).clear);
        assertFalse(FireGate.decide(true, ON_TARGET, TOL, atSpeed("Pollen"), null).clear);
        assertFalse(FireGate.decide(true, ON_TARGET, TOL, (FireGate.Shooter[]) null).clear);
        assertEquals("AIM ERROR UNKNOWN", FireGate.decide(true, Double.NaN, TOL, atSpeed("Pollen")).blockedReason);
    }
}
