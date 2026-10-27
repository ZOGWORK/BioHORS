package org.firstinspires.ftc.teamcode.aim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Carries the alliance and the last pose from the auto to the teleop through the SDK's
 * {@link OpMode#blackboard}, which survives between OpModes until the robot restarts.
 *
 * Pose: field frame (see {@link FieldTargets}), inches and radians.
 */
public final class PoseHandoff {

    private PoseHandoff() {
    }

    private static final String KEY_ALLIANCE = "biohors.alliance";
    private static final String KEY_POSE = "biohors.pose";

    /** Stores only the alliance (the Choose Alliance OpMode). Clears any old pose. */
    public static void saveAlliance(Alliance alliance) {
        OpMode.blackboard.put(KEY_ALLIANCE, alliance.name());
        OpMode.blackboard.remove(KEY_POSE);
    }

    public static void save(Alliance alliance, double x, double y, double heading) {
        OpMode.blackboard.put(KEY_ALLIANCE, alliance.name());
        OpMode.blackboard.put(KEY_POSE, new double[] {x, y, heading});
    }

    /** The alliance the last auto ran on, or null if no auto has saved one. */
    public static Alliance savedAlliance() {
        Object a = OpMode.blackboard.get(KEY_ALLIANCE);
        if (!(a instanceof String)) return null;
        try {
            return Alliance.valueOf((String) a);
        } catch (IllegalArgumentException e) {
            return null;
        }
    }

    /** The last pose the auto saved, {x, y, heading}, or null. */
    public static double[] savedPose() {
        Object p = OpMode.blackboard.get(KEY_POSE);
        if (!(p instanceof double[]) || ((double[]) p).length != 3) return null;
        return ((double[]) p).clone();
    }
}
