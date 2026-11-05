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
    void shootersThatNeverGetToSpeedFireAnywayAtTheTimeout() {
        volley.press();
        assertEquals(0, run(VolleySequence.SPINUP_TIMEOUT_S - 0.1, false));
        assertTrue(volley.running(), "still waiting just before the timeout");
        int requested = run(VolleySequence.FIRE_WINDOW_S + 0.5, false);
        assertEquals(VolleySequence.FIRE_WINDOW_S, requested * TICK, 0.2 + 1.5 * TICK,
                "fires one window anyway once the spin-up time is up");
        assertFalse(volley.running());
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
