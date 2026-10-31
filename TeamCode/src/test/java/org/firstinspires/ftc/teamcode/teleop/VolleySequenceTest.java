package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.SimElapsedTime;
import org.junit.jupiter.api.Test;

/** The Y volley's phases on a simulated clock: spin-up timeout, fire window length, cancel. */
class VolleySequenceTest {

    static final double TICK = 0.02;

    private final FakeTimer clock = new FakeTimer();
    private final VolleySequence volley = new VolleySequence(SimElapsedTime.factory(clock));

    /** Advances seconds of loops with the shooters ready or not; returns the loops it requested the gates. */
    int run(double seconds, boolean ready) {
        int n = (int) Math.round(seconds / TICK);
        int requested = 0;
        for (int i = 0; i < n; i++) {
            clock.advance(TICK);
            volley.update(ready);
            if (volley.requestsGates()) requested++;
        }
        return requested;
    }

    @Test
    void shootersThatNeverGetToSpeedEndItWithNoGateRequested() {
        volley.press();
        assertEquals(0, run(VolleySequence.SPINUP_TIMEOUT_S - 0.1, false));
        assertTrue(volley.running(), "still waiting just before the timeout");
        assertEquals(0, run(0.2, false));
        assertFalse(volley.running(), "given up at the timeout");
    }

    @Test
    void atSpeedItRequestsTheGatesForOneWindowThenStops() {
        volley.press();
        run(0.3, false);
        int requested = run(3.0, true);
        assertEquals(VolleySequence.FIRE_WINDOW_S, requested * TICK, 1.5 * TICK, "one fire window");
        assertFalse(volley.running());
    }

    @Test
    void aSecondPressCancelsAndAThirdStartsAgain() {
        volley.press();
        run(0.1, true);
        assertTrue(volley.requestsGates());
        volley.press();
        assertFalse(volley.running(), "cancelled at once, mid-window");
        volley.press();
        assertEquals(VolleySequence.Phase.SPIN_UP, volley.phase(), "a fresh press starts over from spin-up");
    }
}
