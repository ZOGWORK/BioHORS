package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

/**
 * The teleop's Y volley: MAINHORS26's "intake auto sequence (Y)" on this robot. One press spins
 * both shooters up, then asks for the gates for a timed fire window, then lets go. A press while
 * it runs cancels it.
 *
 * It never opens a gate itself. It only adds a gate REQUEST (the one B makes) during the fire
 * window; the teleop ORs it with the driver's own B request and the fire interlock (aim/FireGate)
 * still decides. It does not aim either: the driver aims with left trigger as usual, and if the
 * robot is off target the gates stay shut and the window passes.
 *
 * MAINHORS26 kicked one ball per press with its claw. This robot has no claw: everything aboard
 * that the intake feeds while the gates are open goes, so a press is a timed window, not one ball.
 *
 * Phases:
 * <ul>
 *   <li>IDLE: nothing.</li>
 *   <li>SPIN_UP: shooters on, waiting for every healthy shooter to reach its target. Not ready
 *       within SPINUP_TIMEOUT_S: back to IDLE, no gate ever requested.</li>
 *   <li>FIRE: gates requested for FIRE_WINDOW_S, then back to IDLE.</li>
 * </ul>
 */
public class VolleySequence {

    public enum Phase { IDLE, SPIN_UP, FIRE }

    /** Longest wait for the shooters to reach speed before the volley is given up, seconds. SET BY HAND. */
    public static double SPINUP_TIMEOUT_S = 2.0;
    /** How long the gates are requested once the shooters are at speed, seconds. SET BY HAND
     *  (MAINHORS26's whole sequence was 1050 ms, from a flywheel that was already spinning). */
    public static double FIRE_WINDOW_S = 1.0;

    private final ElapsedTime timer;
    private Phase phase = Phase.IDLE;

    public VolleySequence() {
        this(ElapsedTime::new);
    }

    /** With a timer factory (tests pass a simulated clock). */
    public VolleySequence(Supplier<ElapsedTime> timers) {
        this.timer = timers.get();
    }

    /** A Y press: starts the volley when idle, cancels it when running. */
    public void press() {
        phase = phase == Phase.IDLE ? Phase.SPIN_UP : Phase.IDLE;
        timer.reset();
    }

    /**
     * Once per loop, after the shooter loops have run.
     *
     * @param shootersReady every healthy shooter is at its target, and at least one is healthy
     */
    public void update(boolean shootersReady) {
        if (phase == Phase.SPIN_UP) {
            if (shootersReady) {
                phase = Phase.FIRE;
                timer.reset();
            } else if (timer.seconds() >= SPINUP_TIMEOUT_S) {
                phase = Phase.IDLE;
            }
        } else if (phase == Phase.FIRE && timer.seconds() >= FIRE_WINDOW_S) {
            phase = Phase.IDLE;
        }
    }

    public Phase phase() {
        return phase;
    }

    public boolean running() {
        return phase != Phase.IDLE;
    }

    /** The shooters are on for the volley (on top of whatever else has them on). */
    public boolean wantsShooters() {
        return running();
    }

    /** The volley's own gate request, ORed with the driver's B request. */
    public boolean requestsGates() {
        return phase == Phase.FIRE;
    }
}
