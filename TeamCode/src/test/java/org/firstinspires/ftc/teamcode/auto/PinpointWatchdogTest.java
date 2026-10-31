package org.firstinspires.ftc.teamcode.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.SimElapsedTime;
import org.junit.jupiter.api.Test;

/** The auto's odometry watchdog: trips on a lost or never-ready Pinpoint, not on a blip. */
class PinpointWatchdogTest {

    static final double TICK = 0.02;
    private final FakeTimer clock = new FakeTimer();
    private boolean ready;
    private final PinpointWatchdog dog = new PinpointWatchdog(() -> ready, SimElapsedTime.factory(clock));

    /** Runs loops with the Pinpoint as given; returns whether it tripped. */
    boolean loops(int n, boolean isReady) {
        ready = isReady;
        boolean t = false;
        for (int i = 0; i < n; i++) {
            t = dog.update();
            clock.advance(TICK);
        }
        return t;
    }

    @Test
    void aHealthyPinpointNeverTrips() {
        dog.start();
        assertFalse(loops(1500, true));
    }

    @Test
    void aShortBlipDoesNotTrip() {
        dog.start();
        loops(10, true);
        assertFalse(loops(PinpointWatchdog.FAULT_LOOPS - 1, false), "one loop short of the limit");
        assertFalse(loops(100, true), "and READY again resets the count");
    }

    @Test
    void aLostPinpointTripsAndStaysTripped() {
        dog.start();
        loops(10, true);
        assertTrue(loops(PinpointWatchdog.FAULT_LOOPS, false));
        assertTrue(loops(100, true), "a pose that came back mid-route is not trusted again");
        assertEquals("Pinpoint not READY: odometry lost", dog.reason());
    }

    @Test
    void stillCalibratingAtPlayIsFineBriefly() {
        dog.start();
        assertFalse(loops((int) (PinpointWatchdog.NEVER_READY_S / TICK) - 2, false), "calibrating, not lost");
        assertFalse(loops(50, true));
    }

    @Test
    void neverReadyTripsAfterTheLimit() {
        dog.start();
        assertTrue(loops((int) (PinpointWatchdog.NEVER_READY_S / TICK) + 2, false));
        assertEquals("Pinpoint never READY after PLAY", dog.reason());
    }
}
