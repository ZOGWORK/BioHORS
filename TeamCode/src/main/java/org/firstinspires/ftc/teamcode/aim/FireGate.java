package org.firstinspires.ftc.teamcode.aim;

import java.util.Locale;

/**
 * The teleop's fire interlock: whether the gates may be open right now, and if not, why.
 *
 * The gates may open only when ALL of these hold:
 *   - the pose is valid (Pinpoint READY, finite numbers);
 *   - the aim is on target: the heading error (to the lead-shifted aim point) is finite and within
 *     the tolerance;
 *   - every shooter that is switched ON is at its target rpm. A shooter that is off is ignored,
 *     but at least one must be on: open gates with every flywheel stopped would feed pollen into
 *     nothing ("NO SHOOTER ON").
 *
 * Fails closed: anything it cannot check (a non-finite heading error or tolerance, a null shooter)
 * blocks. There is deliberately no override. Pure logic, no hardware: the teleop passes the inputs
 * in and drives the gate servos from {@link Decision#clear}.
 */
public final class FireGate {

    private FireGate() {
    }

    /** What the interlock needs to know about one shooter. */
    public static final class Shooter {
        public final String label;
        public final boolean on;
        public final boolean atTarget;
        public final double targetRpm;
        public final double currentRpm;

        public Shooter(String label, boolean on, boolean atTarget, double targetRpm, double currentRpm) {
            this.label = label;
            this.on = on;
            this.atTarget = atTarget;
            this.targetRpm = targetRpm;
            this.currentRpm = currentRpm;
        }
    }

    /** The verdict: clear to fire, or the reasons it is blocked (never empty when blocked). */
    public static final class Decision {
        public final boolean clear;
        /** "" when clear; otherwise every blocking condition, separated by "; ". */
        public final String blockedReason;

        Decision(boolean clear, String blockedReason) {
            this.clear = clear;
            this.blockedReason = blockedReason;
        }
    }

    /**
     * @param poseOk          the pose is usable for aiming
     * @param headingErrorRad heading error to the aim point, radians
     * @param toleranceRad    on-target tolerance, radians (AimController.ON_TARGET_TOLERANCE_RAD)
     * @param shooters        every shooter that feeds through these gates
     */
    public static Decision decide(boolean poseOk, double headingErrorRad, double toleranceRad,
                                  Shooter... shooters) {
        StringBuilder why = new StringBuilder();
        if (!poseOk) {
            append(why, "POSE INVALID");
        } else if (!Double.isFinite(headingErrorRad) || !Double.isFinite(toleranceRad)) {
            append(why, "AIM ERROR UNKNOWN");
        } else if (Math.abs(headingErrorRad) > toleranceRad) {
            append(why, String.format(Locale.ROOT, "AIM %.1f deg off (tol %.1f)",
                    Math.toDegrees(headingErrorRad), Math.toDegrees(toleranceRad)));
        }
        if (shooters == null) {
            append(why, "NO SHOOTER STATE");
        } else {
            boolean anyOn = false;
            boolean anyUnknown = false;
            for (Shooter s : shooters) {
                if (s == null) {
                    anyUnknown = true;
                    append(why, "NO SHOOTER STATE");
                } else {
                    anyOn |= s.on;
                    if (s.on && !s.atTarget) {
                        append(why, String.format(Locale.ROOT, "%s flywheel %+.0f rpm",
                                s.label, s.currentRpm - s.targetRpm));
                    }
                }
            }
            if (!anyOn && !anyUnknown) append(why, "NO SHOOTER ON");
        }
        return why.length() == 0 ? new Decision(true, "") : new Decision(false, why.toString());
    }

    private static void append(StringBuilder sb, String reason) {
        if (sb.length() > 0) sb.append("; ");
        sb.append(reason);
    }
}
