package org.firstinspires.ftc.teamcode.aim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Ball-flight lead (AimController.leadAimPoint). The robot sits 40 in south of a target at the
 * origin, so the true bearing is straight north (+Y); expected values are worked out here by hand.
 */
class AimLeadTest {

    private static final double RX = 0.0, RY = -40.0, TX = 0.0, TY = 0.0;
    private static final double BALL = 100.0; // in/s, a round number for the hand solution

    @Test
    void aStationaryRobotHasNoLead() {
        double[] p = AimController.leadAimPoint(RX, RY, 0.0, 0.0, TX, TY, BALL);
        assertEquals(TX, p[0], 0.0);
        assertEquals(TY, p[1], 0.0);
    }

    @Test
    void strafingTowardPlusXShiftsTheAimPointTowardMinusX() {
        double[] p = AimController.leadAimPoint(RX, RY, 10.0, 0.0, TX, TY, BALL);
        assertTrue(p[0] < TX - 1.0, "aim point should move against the motion (toward -X), x = " + p[0]);
        assertEquals(TY, p[1], 1e-9, "a sideways strafe moves the point only sideways");
        // The ball drifts +X with the robot during flight, so the aim turns left (CCW, positive).
        assertTrue(AimController.leadAngle(RX, RY, TX, TY, p[0], p[1]) > 0.0);
    }

    @Test
    void strafingTowardMinusXShiftsTheAimPointTowardPlusX() {
        double[] p = AimController.leadAimPoint(RX, RY, -10.0, 0.0, TX, TY, BALL);
        assertTrue(p[0] > TX + 1.0, "aim point should move toward +X, x = " + p[0]);
        assertTrue(AimController.leadAngle(RX, RY, TX, TY, p[0], p[1]) < 0.0);
    }

    @Test
    void threeIterationsConvergeOnTheExactSolution() {
        // Exact: aim = (-10 t, 0), |aim - robot| = 100 t  ->  100 t^2 + 1600 = 10000 t^2,
        // t = sqrt(1600 / 9900) s.
        double t = Math.sqrt(1600.0 / 9900.0);
        double[] p = AimController.leadAimPoint(RX, RY, 10.0, 0.0, TX, TY, BALL);
        assertEquals(-10.0 * t, p[0], 0.01, "x after 3 iterations");
        // And the ball does land on the target: it flies to the aim point at BALL in/s while
        // drifting with the robot for the same flight time.
        double flight = Math.hypot(p[0] - RX, p[1] - RY) / BALL;
        assertEquals(TX, p[0] + 10.0 * flight, 0.01);
    }

    @Test
    void anUnsetOrBadBallSpeedGivesExactlyZeroLead() {
        for (double bad : new double[] {0.0, -50.0, Double.NaN, Double.POSITIVE_INFINITY}) {
            double[] p = AimController.leadAimPoint(RX, RY, 30.0, 10.0, TX, TY, bad);
            assertEquals(TX, p[0], 0.0, "ball speed " + bad);
            assertEquals(TY, p[1], 0.0, "ball speed " + bad);
        }
    }

    @Test
    void aNonFiniteVelocityIsTreatedAsZero() {
        double[] p = AimController.leadAimPoint(RX, RY, Double.NaN, 10.0, TX, TY, BALL);
        assertEquals(TX, p[0], 0.0);
        assertEquals(TY, p[1], 0.0);
        p = AimController.leadAimPoint(RX, RY, 10.0, Double.NEGATIVE_INFINITY, TX, TY, BALL);
        assertEquals(TX, p[0], 0.0);
    }

    @Test
    void theLeadIsClampedToTenDegrees() {
        // 50 in/s sideways against a 100 in/s ball would lead by about 30 deg.
        double[] p = AimController.leadAimPoint(RX, RY, 50.0, 0.0, TX, TY, BALL);
        double lead = AimController.leadAngle(RX, RY, TX, TY, p[0], p[1]);
        assertEquals(AimController.MAX_LEAD_RAD, lead, 1e-9);
        assertEquals(Math.toRadians(10.0), AimController.MAX_LEAD_RAD, 1e-12);
        p = AimController.leadAimPoint(RX, RY, -50.0, 0.0, TX, TY, BALL);
        assertEquals(-AimController.MAX_LEAD_RAD, AimController.leadAngle(RX, RY, TX, TY, p[0], p[1]), 1e-9);
    }

    @Test
    void onTopOfTheTargetThereIsNoLead() {
        double[] p = AimController.leadAimPoint(0.3, -0.2, 20.0, 0.0, TX, TY, BALL);
        assertEquals(TX, p[0], 0.0);
        assertEquals(TY, p[1], 0.0);
    }

    @Test
    void driftingTowardTheTargetDoesNotTurnTheAim() {
        double[] p = AimController.leadAimPoint(RX, RY, 0.0, 15.0, TX, TY, BALL);
        assertEquals(0.0, AimController.leadAngle(RX, RY, TX, TY, p[0], p[1]), 1e-9);
        assertTrue(p[1] < TY, "the point moves short of the target along the line");
    }
}
