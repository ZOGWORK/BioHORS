package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.SimElapsedTime;
import org.junit.jupiter.api.Test;

/**
 * IntakeJamGuard against two fake intake motors: a stall reverses for the set time and then pulls
 * in again; a free or a full intake, and the spin-up from rest, never reverse.
 */
class IntakeJamGuardTest {

    static final double TICK = 0.02;
    /** A jammed roller: the load takes 90 % of the motor's torque. PLACEHOLDER (a test load). */
    static final double JAM_TORQUE_FRACTION = -0.9;

    private final FakeTimer clock = new FakeTimer();
    private final FakeMotor l = new FakeMotor(2800.0, 8.0);
    private final FakeMotor r = new FakeMotor(2800.0, 8.0);
    private final IntakeJamGuard guard = new IntakeJamGuard(l, r, SimElapsedTime.factory(clock));

    /** One loop: the guard picks the power, the motors run on it. Returns the power given. */
    double tick(double requested) {
        double p = guard.power(requested);
        l.setPower(p);
        r.setPower(p);
        l.update(TICK);
        r.update(TICK);
        clock.advance(TICK);
        return p;
    }

    void load(double torqueFraction) {
        l.setExternalTorqueFraction(torqueFraction);
        r.setExternalTorqueFraction(torqueFraction);
    }

    static final double DETECT = MechanismConstants.INTAKE_JAM_DETECT_S;
    static final double SETTLE = MechanismConstants.INTAKE_SENSE_SETTLE_S;
    /** Long enough for every reverse of a jam that never clears, and the give-up after them. */
    static final double GIVE_UP_S = (MechanismConstants.INTAKE_JAM_GIVE_UP_REVERSES + 1) * (SETTLE + DETECT
            + MechanismConstants.INTAKE_JAM_REVERSE_S) + 1.0;

    /** Ticks pulling in for seconds; returns the first tick (from 0) that reversed, or -1. */
    int firstReverse(double seconds) {
        int n = (int) Math.round(seconds / TICK);
        for (int i = 0; i < n; i++) if (tick(1.0) < 0) return i;
        return -1;
    }

    @Test
    void aFreeIntakeNeverReverses() {
        assertEquals(-1, firstReverse(DETECT + 3.0));
        assertEquals(0, guard.jams());
    }

    /** The team (2026-11-06): no flipping the intake on a short stall, only one over 5 s. */
    @Test
    void aStallShorterThanTheDetectTimeNeverReverses() {
        assertEquals(5.0, DETECT, 1e-12, "the team's 5 s");
        assertEquals(-1, firstReverse(1.0), "running free first");
        load(JAM_TORQUE_FRACTION);
        assertEquals(-1, firstReverse(DETECT - 0.1), "stalled 4.9 s: still pulling in");
        load(0.0);
        assertEquals(-1, firstReverse(2.0), "it came free on its own: never reversed");
        assertEquals(0, guard.jams());
    }

    @Test
    void aFullRobotIsNotReadAsAJam() {
        load(-MechanismConstants.INTAKE_SLOWDOWN_PER_PIECE * MechanismConstants.HOPPER_CAPACITY);
        assertEquals(-1, firstReverse(DETECT + 3.0), "a full hopper slows the intake, but must not be reversed out");
    }

    @Test
    void aJamFromTheStartWaitsOutTheSpinUpThenReverses() {
        load(JAM_TORQUE_FRACTION);
        int at = firstReverse(SETTLE + DETECT + 1.0);
        double expectS = SETTLE + DETECT;
        assertEquals(expectS, at * TICK, 2.5 * TICK, "reverses once the settle and the detect time are up");
        assertTrue(guard.reversing());
    }

    @Test
    void aJamReversesForTheSetTimeThenPullsInAgain() {
        assertEquals(-1, firstReverse(1.0), "running free first");
        load(JAM_TORQUE_FRACTION);
        int at = firstReverse(DETECT + 1.0);
        assertTrue(at >= 0, "a stall mid-run reverses");
        assertTrue(at * TICK >= MechanismConstants.INTAKE_JAM_DETECT_S - TICK,
                "not before the stall has lasted the detect time: " + at * TICK + " s");

        int reverseTicks = 1; // the tick that started it
        while (tick(1.0) < 0) reverseTicks++;
        assertEquals(MechanismConstants.INTAKE_JAM_REVERSE_S, reverseTicks * TICK, 1.5 * TICK, "reverse length");
        assertFalse(guard.reversing(), "pulling in again");
        assertEquals(1, guard.jams());
    }

    @Test
    void lettingGoEndsAReverseAtOnce() {
        load(JAM_TORQUE_FRACTION);
        assertTrue(firstReverse(SETTLE + DETECT + 1.0) >= 0);
        assertEquals(0.0, tick(0.0), 1e-12, "trigger released: the intake stops, no more reverse");
        assertFalse(guard.reversing());
        assertEquals(-1.0, tick(-1.0), 1e-12, "the driver's own outtake passes straight through");
    }

    /** Ticks pulling in for seconds; returns how many ticks reversed. */
    int reverseTicks(double seconds) {
        int n = (int) Math.round(seconds / TICK);
        int r = 0;
        for (int i = 0; i < n; i++) if (tick(1.0) < 0) r++;
        return r;
    }

    @Test
    void aJamThatNeverClearsGivesUpAfterTheSetReversesUntilLetGo() {
        load(JAM_TORQUE_FRACTION);
        reverseTicks(GIVE_UP_S);
        assertEquals(MechanismConstants.INTAKE_JAM_GIVE_UP_REVERSES, guard.jams(), "the set number of reverses, then no more");
        assertTrue(guard.givenUp());
        assertEquals(0.0, tick(1.0), 1e-12, "given up: stopped, still pulling the trigger");
        assertEquals(0.0, tick(0.0), 1e-12, "trigger released");
        assertFalse(guard.givenUp(), "letting go clears the give-up");
        assertEquals(1.0, tick(1.0), 1e-12, "pulling again runs in again");
    }

    @Test
    void aReverseThatClearedTheJamDoesNotCountTowardGivingUp() {
        load(JAM_TORQUE_FRACTION);
        while (guard.jams() < MechanismConstants.INTAKE_JAM_GIVE_UP_REVERSES - 1) tick(1.0);
        while (guard.reversing()) tick(1.0);
        load(0.0);                      // that reverse spat it out
        assertEquals(0, reverseTicks(1.5), "running free again");
        load(JAM_TORQUE_FRACTION);      // a new jam, trigger held throughout
        reverseTicks(GIVE_UP_S);
        assertEquals(2 * MechanismConstants.INTAKE_JAM_GIVE_UP_REVERSES - 1, guard.jams(),
                "the new jam gets the full set of reverses");
        assertTrue(guard.givenUp());
    }

    @Test
    void aHoldIsNotARelease() {
        load(JAM_TORQUE_FRACTION);
        reverseTicks(GIVE_UP_S);
        assertTrue(guard.givenUp());
        for (int i = 0; i < 25; i++) assertEquals(0.0, guard.hold(), 1e-12);
        assertTrue(guard.givenUp(), "the teleop stopping the intake does not clear the give-up");
        assertEquals(0.0, tick(1.0), 1e-12, "trigger still held: still stopped");
    }
}
