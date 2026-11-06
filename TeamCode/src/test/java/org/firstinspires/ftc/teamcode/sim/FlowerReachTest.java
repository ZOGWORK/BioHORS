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

    private static boolean reaches(double mx, double my, double headingRad) {
        return BiobuzzMatch.reachesOpening(OPEN_X, OPEN_Y, IN_X, IN_Y, DEPTH_IN, mx, my,
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
        assertTrue(reaches(-24.0 + 1.99, OPEN_Y + 2.0, north), "inside, 1.99 in across");
        assertFalse(reaches(-24.0 + 2.01, OPEN_Y + 2.0, north), "inside, 2.01 in across");
        assertFalse(reaches(-24.0, OPEN_Y + 2.0, north + Math.toRadians(30)), "inside, turned 30 deg");
        assertFalse(reaches(-24.0, OPEN_Y - 1.0, -north), "at the face, facing away");
    }
}
