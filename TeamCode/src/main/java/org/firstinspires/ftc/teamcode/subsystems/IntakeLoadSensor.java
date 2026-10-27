package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

/**
 * Counts the pollen aboard from how much the intake slows down. Pollen held in the robot press
 * on the intake rollers, so with the intake running inward at full power the two motors turn
 * slower the more pollen there are. Both intake motors have encoders; their speeds are averaged.
 *
 * The count is only read while the intake runs inward at INTAKE_SENSE_MIN_POWER or more, after it
 * has run for INTAKE_SENSE_SETTLE_S (the start-up transient looks like load), and a new count must
 * hold for INTAKE_SENSE_DEBOUNCE_S before it is believed. Otherwise the last count is kept, except
 * that running the intake outward (ejecting) forgets it.
 *
 * Calibrate on the robot (MechanismConstants, PLACEHOLDER): run the intake empty and read
 * "Intake speed" for INTAKE_FREE_TICKS_PER_S; then hold 1, 2, 3, 4 pollen and read the slowdown
 * for INTAKE_SLOWDOWN_PER_PIECE.
 */
public class IntakeLoadSensor {

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final ElapsedTime running;
    private final ElapsedTime candidateHeld;

    private boolean wasRunning;
    private double filteredTicksPerS = Double.NaN;
    private int count = -1;        // -1: not read yet
    private int candidate = -1;
    private long readings;         // settled loops read so far (lets callers ask for a fresh one)

    public IntakeLoadSensor(DcMotorEx left, DcMotorEx right) {
        this(left, right, ElapsedTime::new);
    }

    /** With a timer factory (tests pass a simulated clock). */
    public IntakeLoadSensor(DcMotorEx left, DcMotorEx right, Supplier<ElapsedTime> timers) {
        this.left = left;
        this.right = right;
        this.running = timers.get();
        this.candidateHeld = timers.get();
    }

    /** Once per loop, with the power the intake was just given. */
    public void update(double intakePower) {
        // Running outward ejects pollen: whatever was counted is no longer known.
        if (intakePower < -0.1) {
            count = -1;
            candidate = -1;
        }
        boolean runningIn = intakePower >= MechanismConstants.INTAKE_SENSE_MIN_POWER;
        if (runningIn && !wasRunning) {
            running.reset();
            filteredTicksPerS = Double.NaN;
            candidate = -1;
        }
        wasRunning = runningIn;
        if (!runningIn) return;

        double v = (Math.abs(left.getVelocity()) + Math.abs(right.getVelocity())) / 2.0;
        double a = MechanismConstants.INTAKE_SENSE_FILTER_ALPHA;
        filteredTicksPerS = Double.isNaN(filteredTicksPerS) ? v : a * v + (1 - a) * filteredTicksPerS;
        if (running.seconds() < MechanismConstants.INTAKE_SENSE_SETTLE_S) return;
        readings++;

        int now = countFor(filteredTicksPerS);
        if (now != candidate) {
            candidate = now;
            candidateHeld.reset();
        } else if (candidateHeld.seconds() >= MechanismConstants.INTAKE_SENSE_DEBOUNCE_S) {
            count = candidate;
        }
    }

    /** Fraction of the empty free speed lost (0 = empty, about 0.06 per pollen by default). */
    public double slowdown() {
        if (Double.isNaN(filteredTicksPerS)) return 0;
        return Math.max(0, 1.0 - filteredTicksPerS / MechanismConstants.INTAKE_FREE_TICKS_PER_S);
    }

    /**
     * The count for an intake speed. FULL only at or below the loaded threshold
     * (INTAKE_LOADED_TICKS_PER_S, MainHORS's); above it, the nearest whole number of pollen, at most
     * one short of full.
     */
    static int countFor(double ticksPerS) {
        int capacity = MechanismConstants.HOPPER_CAPACITY;
        if (ticksPerS <= MechanismConstants.INTAKE_LOADED_TICKS_PER_S) return capacity;
        double slowdown = Math.max(0, 1.0 - ticksPerS / MechanismConstants.INTAKE_FREE_TICKS_PER_S);
        int n = (int) Math.floor(slowdown / MechanismConstants.INTAKE_SLOWDOWN_PER_PIECE + 0.5);
        return Math.max(0, Math.min(capacity - 1, n));
    }

    /** The pollen count last believed, or -1 if the intake has not run long enough to tell. */
    public int count() {
        return count;
    }

    public boolean hasReading() {
        return count >= 0;
    }

    /** How many settled loops have been read so far; compare two values to see a fresh reading. */
    public long readings() {
        return readings;
    }

    /** The volley emptied the robot: start the count again from nothing. */
    public void emptied() {
        count = 0;
        candidate = -1;
    }

    /** Average intake speed, ticks/s (for telemetry and calibration). */
    public double speedTicksPerS() {
        return Double.isNaN(filteredTicksPerS) ? 0 : filteredTicksPerS;
    }
}
