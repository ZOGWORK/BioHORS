package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.aim.Alliance;

/**
 * Where the autos may shoot from: the scoring spots (AutoPoses.SCORE_SOUTH and SCORE_NORTH, the
 * Zenith draft's scoreSouth and scoreNorth), as seen by the alliance. Both autos open the pollen
 * gate only while the robot is at one of them; a volley that never gets there is skipped, so a
 * ball is never fed from a pose the shot was not planned from.
 */
public final class ShootingPose {

    // SET BY HAND: the shot from 48 in seats over a few inches of range (ShotSweepTest: 46-50 in),
    // and the follower ends well inside this.
    public static double POSITION_TOLERANCE_IN = 2.0;
    // SET BY HAND: the same as the follower's heading deviation tolerance (pedro/Constants, 3 deg).
    public static double HEADING_TOLERANCE_RAD = Math.toRadians(3.0);
    // PLACEHOLDER: how long a volley waits at speed for the robot to reach a spot before it is skipped,
    // and how long a volley that is knocked off its spot waits to get back before it ends.
    public static double WAIT_FOR_POSE_S = 1.0;

    private static final Pose[] SPOTS = {AutoPoses.SCORE_SOUTH, AutoPoses.SCORE_NORTH};

    private ShootingPose() {
    }

    /** Pollen a volley of count really fed, from the seconds the gate was open and feeding. */
    public static int fedPieces(double fedSeconds, int count, double secondsPerPiece, double fullVolleySeconds) {
        if (fedSeconds >= fullVolleySeconds) return count;
        return Math.max(0, Math.min(count, (int) Math.round(fedSeconds / secondsPerPiece)));
    }

    /** Whether the robot (field frame, inches and radians) is at one of the alliance's scoring spots. */
    public static boolean at(Alliance alliance, Pose robot) {
        if (robot == null) return false;
        for (Pose red : SPOTS) {
            Pose spot = AutoPoses.of(alliance, red);
            double dHeading = Math.atan2(Math.sin(robot.heading() - spot.heading()),
                    Math.cos(robot.heading() - spot.heading()));
            if (Math.hypot(robot.x() - spot.x(), robot.y() - spot.y()) <= POSITION_TOLERANCE_IN
                    && Math.abs(dHeading) <= HEADING_TOLERANCE_RAD) {
                return true;
            }
        }
        return false;
    }
}
