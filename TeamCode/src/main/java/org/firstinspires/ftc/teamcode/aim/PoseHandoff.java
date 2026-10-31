package org.firstinspires.ftc.teamcode.aim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Carries the alliance and the last pose from the auto to the teleop through the SDK's
 * {@link OpMode#blackboard}, which survives between OpModes until the robot restarts.
 *
 * Pose: field frame (see {@link FieldTargets}), inches and radians.
 *
 * The pose expires: it is only handed over if it was saved less than MAX_POSE_AGE_S ago (a timer
 * saved with it; ElapsedTime, so SimLoop runs it on sim time), and the teleop takes it once
 * ({@link #clearPose()}). Otherwise a practice auto's pose, run long before, would be trusted by a
 * later teleop after the robot had been carried somewhere else.
 */
public final class PoseHandoff {

    private PoseHandoff() {
    }

    private static final String KEY_ALLIANCE = "biohors.alliance";
    private static final String KEY_POSE = "biohors.pose";
    private static final String KEY_POSE_TIMER = "biohors.poseTimer";

    /**
     * A saved pose older than this, seconds, is not handed over. SET BY HAND: FTC's transition from
     * AUTO to the teleop INIT takes seconds, so a minute only throws away a pose from a different run.
     * Throwing a good one away costs little: the teleop then assumes the auto's park spot.
     */
    public static double MAX_POSE_AGE_S = 60.0;

    /** Stores only the alliance (the Choose Alliance OpMode). Clears any old pose. */
    public static void saveAlliance(Alliance alliance) {
        OpMode.blackboard.put(KEY_ALLIANCE, alliance.name());
        clearPose();
    }

    public static void save(Alliance alliance, double x, double y, double heading) {
        OpMode.blackboard.put(KEY_ALLIANCE, alliance.name());
        OpMode.blackboard.put(KEY_POSE, new double[] {x, y, heading});
        OpMode.blackboard.put(KEY_POSE_TIMER, new ElapsedTime());
    }

    /** Forgets the saved pose (the teleop calls this once it has taken it). The alliance stays. */
    public static void clearPose() {
        OpMode.blackboard.remove(KEY_POSE);
        OpMode.blackboard.remove(KEY_POSE_TIMER);
    }

    /** Seconds since the pose was saved, or NaN if there is no saved pose with a timer. */
    public static double savedPoseAgeS() {
        Object t = OpMode.blackboard.get(KEY_POSE_TIMER);
        if (!(t instanceof ElapsedTime) || rawPose() == null) return Double.NaN;
        return ((ElapsedTime) t).seconds();
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

    /**
     * The last pose the auto saved, {x, y, heading}, or null if there is none, it has no timer, or it
     * is older than MAX_POSE_AGE_S.
     */
    public static double[] savedPose() {
        double age = savedPoseAgeS();
        if (!(age >= 0.0 && age <= MAX_POSE_AGE_S)) return null; // NaN fails too
        return rawPose().clone();
    }

    private static double[] rawPose() {
        Object p = OpMode.blackboard.get(KEY_POSE);
        if (!(p instanceof double[]) || ((double[]) p).length != 3) return null;
        return (double[]) p;
    }
}
