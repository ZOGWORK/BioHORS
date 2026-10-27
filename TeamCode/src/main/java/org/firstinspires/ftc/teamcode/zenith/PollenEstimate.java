package org.firstinspires.ftc.teamcode.zenith;

/**
 * How many pollen the robot THINKS it holds. This robot has no sensor that sees pollen, so this is
 * an estimate, not a reading:
 *   - it starts at the preload (4, the BIOBUZZ maximum, confirmed for this robot);
 *   - every INTAKE_S_PER_PIECE seconds the intake runs inward counts as one more pollen;
 *   - a score of N takes N away.
 *
 * NEEDS A SENSOR: a beam break or colour sensor in the hopper would replace all of this. Until then
 * the hopperFull / holdingPiece / hopperEmpty conditions are only as good as the timing below, and
 * the SimLoop match test reports how far the estimate ended from the simulated truth.
 */
public class PollenEstimate {

    public static final int CAPACITY = 4; // SET BY HAND: confirmed by Sushil (Oct 27); also BIOBUZZ G407
    /** Seconds of inward intake per pollen taken in. PLACEHOLDER: NEEDS MEASUREMENT. */
    public static double INTAKE_S_PER_PIECE = 0.35;

    private double held;

    public PollenEstimate(int preload) {
        held = Math.max(0, Math.min(CAPACITY, preload));
    }

    /** Call every loop with how long the intake ran inward this loop. */
    public void intakeRan(double seconds) {
        if (seconds > 0) held = Math.min(CAPACITY, held + seconds / INTAKE_S_PER_PIECE);
    }

    public void launched(int count) {
        held = Math.max(0, held - count);
    }

    public int held() {
        return (int) Math.floor(held + 1e-9);
    }

    public boolean full() {
        return held() >= CAPACITY;
    }

    public boolean empty() {
        return held() <= 0;
    }
}
