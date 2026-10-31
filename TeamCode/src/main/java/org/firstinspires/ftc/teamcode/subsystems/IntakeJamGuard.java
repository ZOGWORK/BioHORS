package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

/**
 * Teleop jam clearing for the intake. While the driver pulls in, a stalled intake (both motors'
 * average speed below INTAKE_JAM_TICKS_PER_S for INTAKE_JAM_DETECT_S) is run outward for
 * INTAKE_JAM_REVERSE_S, then pulls in again. Letting go of intake ends a reverse at once.
 *
 * The speed is only judged after the intake has run inward for INTAKE_SENSE_SETTLE_S (the spin-up
 * from rest, and from a reverse, is slow on purpose). The jam threshold sits far below a FULL robot's
 * speed (INTAKE_LOADED_TICKS_PER_S), so a full hopper is not reversed out. Thresholds: PLACEHOLDER
 * (MechanismConstants).
 *
 * Give-up: after INTAKE_JAM_GIVE_UP_REVERSES reverses in a row that did not clear it (the intake
 * never got back up to speed between them), the next stall stops the intake instead of grinding
 * the stuck ball. It stays stopped until the driver lets go of intake; pulling in again starts over.
 */
public class IntakeJamGuard {

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final ElapsedTime runningIn;
    private final ElapsedTime stalled;
    private final ElapsedTime reversing;

    private boolean wasIn;
    private boolean stallSeen;
    private boolean isReversing;
    private int jams;
    private int reversesInARow;
    private boolean givenUp;

    public IntakeJamGuard(DcMotorEx left, DcMotorEx right) {
        this(left, right, ElapsedTime::new);
    }

    /** With a timer factory (tests pass a simulated clock). */
    public IntakeJamGuard(DcMotorEx left, DcMotorEx right, Supplier<ElapsedTime> timers) {
        this.left = left;
        this.right = right;
        this.runningIn = timers.get();
        this.stalled = timers.get();
        this.reversing = timers.get();
    }

    /**
     * Once per loop, with the power the driver asks for (+ = in). Returns the power to give the
     * intake: the request, or the outtake power while a jam reverse runs.
     */
    public double power(double requested) {
        boolean wantIn = requested >= MechanismConstants.INTAKE_SENSE_MIN_POWER;
        if (isReversing) {
            if (wantIn && reversing.seconds() < MechanismConstants.INTAKE_JAM_REVERSE_S) {
                return IntakeController.OUTTAKE_POWER;
            }
            isReversing = false;
            wasIn = false; // pulling in again starts a fresh spin-up
        }
        if (!wantIn) {
            wasIn = false;
            reversesInARow = 0;
            givenUp = false;
            return requested;
        }
        if (givenUp) return 0.0;
        if (!wasIn) {
            runningIn.reset();
            stallSeen = false;
            wasIn = true;
        }
        if (runningIn.seconds() < MechanismConstants.INTAKE_SENSE_SETTLE_S) return requested;

        double v = (Math.abs(left.getVelocity()) + Math.abs(right.getVelocity())) / 2.0;
        if (v >= MechanismConstants.INTAKE_JAM_TICKS_PER_S) {
            stallSeen = false;
            reversesInARow = 0;   // back up to speed: the last reverse cleared it
            return requested;
        }
        if (!stallSeen) {
            stallSeen = true;
            stalled.reset();
        }
        if (stalled.seconds() < MechanismConstants.INTAKE_JAM_DETECT_S) return requested;

        if (reversesInARow >= MechanismConstants.INTAKE_JAM_GIVE_UP_REVERSES) {
            givenUp = true;
            return 0.0;
        }
        reversesInARow++;
        isReversing = true;
        reversing.reset();
        jams++;
        return IntakeController.OUTTAKE_POWER;
    }

    /**
     * In place of {@link #power} for a loop where the teleop, not the driver, stops the intake (the
     * Y volley's spin-up, with the trigger still held). Returns 0. The next pull starts a fresh
     * spin-up, but this is not a release: a give-up and the count of reverses in a row stand.
     */
    public double hold() {
        isReversing = false;
        wasIn = false;
        return 0.0;
    }

    /** True while a jam reverse is running. */
    public boolean reversing() {
        return isReversing;
    }

    /** True once it has given up on a jam: the intake is held stopped until the driver lets go. */
    public boolean givenUp() {
        return givenUp;
    }

    /** Jam reverses started since the OpMode began. */
    public int jams() {
        return jams;
    }
}
