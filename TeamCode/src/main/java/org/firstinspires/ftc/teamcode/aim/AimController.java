package org.firstinspires.ftc.teamcode.aim;

/**
 * Turns the whole chassis to face a field point. There is no turret, so aiming is chassis yaw.
 *
 * Inputs are the robot's field pose (inches, heading radians CCW from +X) and a target point.
 * Output is a yaw command for DriveController, whose yaw is POSITIVE CLOCKWISE, so a positive
 * (CCW) heading error gives a negative yaw command.
 *
 * Pure math, no hardware: the teleop decides whether the pose is trustworthy before using it.
 */
public class AimController {

    // Which way the shooter fires, relative to the robot's front, radians CCW.
    // 0 = the shooter fires out of the robot's front. NEEDS MEASUREMENT: confirm on the robot.
    public static double SHOOTER_FACING_OFFSET_RAD = 0.0; // PLACEHOLDER

    public static double kP = 1.2;                  // PLACEHOLDER: yaw command per radian of error
    public static double MAX_YAW = 0.7;             // PLACEHOLDER: cap on the aim yaw command
    public static double MIN_YAW = 0.06;            // PLACEHOLDER: static-friction floor outside tolerance
    public static double ON_TARGET_TOLERANCE_RAD = Math.toRadians(2.0); // PLACEHOLDER
    // Chassis turn rate at full yaw command, rad/s. Turns the bearing-rate feedforward into a yaw
    // command. PLACEHOLDER: NEEDS MEASUREMENT (spin in place at full yaw, read the Pinpoint).
    public static double MAX_TURN_RATE_RAD_PER_S = 5.0;

    private double lastHeadingErrorRad = 0.0;

    /** Field bearing from the robot to the target, radians CCW from +X. */
    public static double bearingTo(double robotX, double robotY, double targetX, double targetY) {
        return Math.atan2(targetY - robotY, targetX - robotX);
    }

    /** Heading the robot must hold so the shooter faces the target. */
    public static double desiredHeading(double robotX, double robotY, double targetX, double targetY) {
        return bearingTo(robotX, robotY, targetX, targetY) - SHOOTER_FACING_OFFSET_RAD;
    }

    /** Wraps an angle to (-pi, pi]. */
    public static double wrap(double rad) {
        double a = rad % (2.0 * Math.PI);
        if (a <= -Math.PI) a += 2.0 * Math.PI;
        if (a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    /**
     * Rate the field bearing to the target turns at because the robot is moving, rad/s CCW.
     * vx, vy: robot field velocity, in/s.
     */
    public static double bearingRate(double robotX, double robotY, double vx, double vy,
                                     double targetX, double targetY) {
        double rx = targetX - robotX;
        double ry = targetY - robotY;
        double r2 = rx * rx + ry * ry;
        if (r2 < 1.0) return 0.0; // on top of the target: the bearing is meaningless
        return (ry * vx - rx * vy) / r2;
    }

    /** Yaw command for a stationary robot. */
    public double yawCommand(double robotX, double robotY, double robotHeading,
                             double targetX, double targetY) {
        return yawCommand(robotX, robotY, robotHeading, 0.0, 0.0, targetX, targetY);
    }

    /**
     * Yaw command (DriveController units, positive clockwise) that turns the robot toward the
     * target: proportional on the heading error, plus a feedforward for how fast the bearing is
     * turning because the robot is driving (so aim keeps up while the driver strafes). Also
     * records the heading error for telemetry.
     *
     * @param vx robot field velocity x, in/s
     * @param vy robot field velocity y, in/s
     */
    public double yawCommand(double robotX, double robotY, double robotHeading, double vx, double vy,
                             double targetX, double targetY) {
        double error = wrap(desiredHeading(robotX, robotY, targetX, targetY) - robotHeading);
        lastHeadingErrorRad = error;
        double ff = bearingRate(robotX, robotY, vx, vy, targetX, targetY) / MAX_TURN_RATE_RAD_PER_S;

        double ccw;
        if (Math.abs(error) <= ON_TARGET_TOLERANCE_RAD) {
            ccw = ff;
        } else {
            double p = kP * error;
            p = Math.signum(p) * Math.max(MIN_YAW, Math.abs(p));
            ccw = p + ff;
        }
        ccw = Math.max(-MAX_YAW, Math.min(MAX_YAW, ccw));
        return -ccw; // CCW-positive -> DriveController's clockwise-positive yaw
    }

    /** Heading error from the last {@link #yawCommand} call, radians CCW. */
    public double getLastHeadingErrorRad() {
        return lastHeadingErrorRad;
    }

    public boolean isOnTarget() {
        return Math.abs(lastHeadingErrorRad) <= ON_TARGET_TOLERANCE_RAD;
    }
}
