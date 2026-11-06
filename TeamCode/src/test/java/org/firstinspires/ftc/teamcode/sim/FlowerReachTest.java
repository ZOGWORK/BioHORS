package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * BiobuzzMatch's flower reach on its own: flower1's opening as the field file has it (the foot's face
 * at y = 67.1, the opening 3.57 in deep, its back the wall), the robot facing north into it.
 */
class FlowerReachTest {

    static final double OPEN_X = -24.0, OPEN_Y = 67.1, IN_X = 0.0, IN_Y = -1.0;   // SPEC: biobuzz.field.json
    static final double DEPTH_IN = 3.57;                                           // SPEC: flower1's openingIn.depthIn
    // MEASURED FROM CAD: the 13.2 in mouth (robot.json) spans flower1's 6 in opening (SPEC) up to 3.6 in off-centre.
    static final double ACROSS_IN = 13.2 / 2 - 6.0 / 2;

    private static boolean reaches(double mx, double my, double headingRad) {
        return BiobuzzMatch.reachesOpening(OPEN_X, OPEN_Y, IN_X, IN_Y, DEPTH_IN, ACROSS_IN, mx, my,
                Math.cos(headingRad), Math.sin(headingRad));
    }

    @Test
    void aTipInsideTheOpeningReachesAndOnePastItsBackDoesNot() {
        double north = Math.PI / 2;
        assertTrue(reaches(-24.0, OPEN_Y + 3.09, north), "the pickup's tip, 3.09 in in");
        assertTrue(reaches(-24.0, OPEN_Y + DEPTH_IN - 0.01, north), "just inside the back");
        assertFalse(reaches(-24.0, OPEN_Y + DEPTH_IN + 0.01, north), "just past the back (the wall)");
    }

    @Test
    void outOnTheFieldTheReachIsTwoInchesFromTheFace() {
        double north = Math.PI / 2;
        assertTrue(reaches(-24.0, OPEN_Y - 1.99, north), "1.99 in short of the face");
        assertFalse(reaches(-24.0, OPEN_Y - 2.01, north), "2.01 in short of the face");
    }

    @Test
    void aTipOffToTheSideOrARobotFacingAwayDoesNotReach() {
        double north = Math.PI / 2;
        assertTrue(reaches(-24.0 + 3.0, OPEN_Y + 2.0, north), "inside, 3.0 in across (the pickup, 3 in right)");
        assertTrue(reaches(-24.0 - 3.59, OPEN_Y + 2.0, north), "inside, 3.59 in across");
        assertFalse(reaches(-24.0 + 3.61, OPEN_Y + 2.0, north), "inside, 3.61 in across: the mouth no longer spans it");
        assertFalse(reaches(-24.0, OPEN_Y + 2.0, north + Math.toRadians(30)), "inside, turned 30 deg");
        assertFalse(reaches(-24.0, OPEN_Y - 1.0, -north), "at the face, facing away");
    }
}
