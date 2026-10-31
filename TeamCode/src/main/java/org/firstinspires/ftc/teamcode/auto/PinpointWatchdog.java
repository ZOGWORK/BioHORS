package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Halts an auto whose odometry has failed. Pedro follows the Pinpoint's pose; if the Pinpoint
 * faults mid-route the pose freezes and the follower drives on blind, into a wall or the hive.
 *
 * Trips (and stays tripped for the rest of the auto) when, after PLAY:
 *   - the Pinpoint has been READY and then is not READY for FAULT_LOOPS loops in a row, or
 *   - it has never been READY NEVER_READY_S after PLAY (still calibrating at PLAY is fine, briefly).
 *
 * Both autos call {@link #update()} once per loop and, when it returns true, stop the drive and keep
 * the gates shut (the scoring-spot check cannot be trusted without a pose).
 */
public class PinpointWatchdog {

    /** Not READY this many loops in a row trips it. SET BY HAND: rides out one bad read. */
    public static int FAULT_LOOPS = 5;
    /** Never READY this long after PLAY trips it, seconds. SET BY HAND: the Pinpoint calibrates in a
     *  fraction of a second, so a full second is a dead or unplugged one. */
    public static double NEVER_READY_S = 1.0;

    private final BooleanSupplier ready;
    private final ElapsedTime sinceStart;
    private boolean seenReady;
    private int notReadyLoops;
    private boolean tripped;
    private String reason = "";

    public PinpointWatchdog(BooleanSupplier ready) {
        this(ready, ElapsedTime::new);
    }

    /** With a timer factory (tests pass a simulated clock). */
    public PinpointWatchdog(BooleanSupplier ready, Supplier<ElapsedTime> timers) {
        this.ready = ready;
        this.sinceStart = timers.get();
    }

    /** Call at PLAY: the never-READY timer starts here. */
    public void start() {
        sinceStart.reset();
    }

    /** Once per loop after PLAY. Returns true once tripped (and every loop after). */
    public boolean update() {
        if (tripped) return true;
        if (ready.getAsBoolean()) {
            seenReady = true;
            notReadyLoops = 0;
            return false;
        }
        if (seenReady) {
            if (++notReadyLoops >= FAULT_LOOPS) trip("Pinpoint not READY: odometry lost");
        } else if (sinceStart.seconds() >= NEVER_READY_S) {
            trip("Pinpoint never READY after PLAY");
        }
        return tripped;
    }

    private void trip(String why) {
        tripped = true;
        reason = why;
    }

    public boolean tripped() {
        return tripped;
    }

    /** Why it tripped, for telemetry; "" if it has not. */
    public String reason() {
        return reason;
    }
}
