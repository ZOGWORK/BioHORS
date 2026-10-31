package org.firstinspires.ftc.teamcode.aim;

/**
 * Catches a pose the robot cannot really have. The Pinpoint can report READY and still give a bad
 * pose (a pod knocked loose, a glitch, a wrong reset), and the teleop would aim and fire on it.
 *
 * A pose fails when it is more than OFF_FIELD_MARGIN_IN outside the field, or when it moved further
 * in one loop than the robot can drive (top speed x dt x JUMP_SPEED_FACTOR + JUMP_MARGIN_IN). A
 * failure is latched: the pose counts as invalid until {@link #reset()} (the teleop's X reset).
 * Field frame, inches. Pure logic: the teleop passes the pose and the loop's dt.
 */
public class PoseSanity {

    /** Half the field, inches: the walls are about 70.7 in from the centre (FIRST's CAD). APPROX. */
    public static double FIELD_HALF_IN = 70.7;
    /** How far past a wall a pose may be before it is impossible, inches. SET BY HAND: the robot's
     *  centre can never be past a wall, so this only absorbs odometry error. */
    public static double OFF_FIELD_MARGIN_IN = 6.0;
    /** Slack on the top-speed travel per loop. SET BY HAND. */
    public static double JUMP_SPEED_FACTOR = 1.5;
    /** Extra distance allowed per loop, inches. SET BY HAND. */
    public static double JUMP_MARGIN_IN = 3.0;

    private final double maxSpeedInPerS;
    private double lastX = Double.NaN;
    private double lastY = Double.NaN;
    private String fault = "";

    /** @param maxSpeedInPerS the robot's top speed, in/s */
    public PoseSanity(double maxSpeedInPerS) {
        this.maxSpeedInPerS = maxSpeedInPerS;
    }

    /** Once per loop with the pose and the loop's dt. Returns true while the pose is believable. */
    public boolean check(double x, double y, double dtS) {
        if (!fault.isEmpty()) return false;
        if (!Double.isFinite(x) || !Double.isFinite(y)) return false; // not a number: not ours to latch
        double limit = FIELD_HALF_IN + OFF_FIELD_MARGIN_IN;
        if (Math.abs(x) > limit || Math.abs(y) > limit) {
            fault = "OFF FIELD";
        } else if (Double.isFinite(lastX) && dtS > 0) {
            double moved = Math.hypot(x - lastX, y - lastY);
            if (moved > maxSpeedInPerS * dtS * JUMP_SPEED_FACTOR + JUMP_MARGIN_IN) fault = "POSE JUMPED";
        }
        lastX = x;
        lastY = y;
        return fault.isEmpty();
    }

    /** Trust the pose again (after the driver reset it); the next pose starts a fresh track. */
    public void reset() {
        fault = "";
        lastX = Double.NaN;
        lastY = Double.NaN;
    }

    /** Why the pose is not believed ("" if it is). */
    public String fault() {
        return fault;
    }
}
