package org.firstinspires.ftc.teamcode.aim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** A pose off the field, or one that moved further than the robot can drive, is latched invalid. */
class PoseSanityTest {

    static final double TOP_IN_PER_S = 60.0;   // a test robot's top speed
    static final double TICK = 0.02;

    @Test
    void drivingFlatOutAcrossTheFieldIsBelieved() {
        PoseSanity s = new PoseSanity(TOP_IN_PER_S);
        for (int i = 0; i <= 100; i++) {
            assertTrue(s.check(-60.0 + i * TOP_IN_PER_S * TICK, 10.0, i * TICK), "tick " + i);
        }
        assertEquals("", s.fault());
    }

    @Test
    void aJumpIsLatchedUntilReset() {
        PoseSanity s = new PoseSanity(TOP_IN_PER_S);
        assertTrue(s.check(0.0, 0.0, 0.0));
        assertFalse(s.check(20.0, 0.0, TICK), "20 in in one 20 ms loop");
        assertEquals("POSE JUMPED", s.fault());
        assertFalse(s.check(20.0, 0.0, 2 * TICK), "still not believed, though it is now still");
        s.reset();
        assertTrue(s.check(20.0, 0.0, 3 * TICK), "after the X reset");
    }

    @Test
    void offTheFieldIsNotBelieved() {
        PoseSanity s = new PoseSanity(TOP_IN_PER_S);
        double edge = PoseSanity.FIELD_HALF_IN + PoseSanity.OFF_FIELD_MARGIN_IN;
        assertTrue(s.check(edge - 0.5, 0.0, 0.0), "just inside the margin");
        s.reset();
        assertFalse(s.check(0.0, -(edge + 0.5), TICK));
        assertEquals("OFF FIELD", s.fault());
    }

    @Test
    void aSlowLoopAllowsMoreTravel() {
        PoseSanity s = new PoseSanity(TOP_IN_PER_S);
        assertTrue(s.check(0.0, 0.0, 0.0));
        assertTrue(s.check(10.0, 0.0, 0.15), "10 in over a 150 ms loop is drivable");
    }

    @Test
    void drivingWhileThePoseWasUnusableIsNotAJump() {
        // The pose was unusable (not checked) for a second while the driver drove 40 in by hand;
        // the next usable pose is 40 in on. Against one loop's 20 ms that would be a jump; against
        // the second since the last checked pose it is ordinary driving.
        PoseSanity s = new PoseSanity(TOP_IN_PER_S);
        assertTrue(s.check(0.0, 0.0, 0.0));
        assertTrue(s.check(40.0, 0.0, 1.0), "40 in after 1 s unchecked: drivable");
        assertEquals("", s.fault());
    }
}
