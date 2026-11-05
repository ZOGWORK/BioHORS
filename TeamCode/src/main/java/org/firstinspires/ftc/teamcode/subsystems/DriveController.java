package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Mecanum drive with HORS-style input shaping: deadband + expo curves on the
 * sticks, then standard mecanum mixing with normalization and a speed scale.
 *
 * Shaping values below are STARTING POINTS for this robot, not tuned values.
 * If the drive feels twitchy or dead near center, tune STICK_DEADBAND,
 * TRANSLATION_EXPO, and YAW_EXPO here.
 */
public class DriveController {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    /** A power change smaller than this is not written: each motor write is a hub command of
     *  about a millisecond, and while aiming the yaw changes a hair every loop. A move to exactly
     *  zero is always written. SET BY HAND. */
    public static double POWER_WRITE_EPSILON = 0.005;
    // Last power written to frontLeft, frontRight, backLeft, backRight; NaN = nothing written yet.
    private final double[] lastPower = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    // ===== Drive shaping config (starting points, tune to this robot) =====
    public static double STICK_DEADBAND = 0.05; // PLACEHOLDER
    public static double TRANSLATION_EXPO = 1.6; // PLACEHOLDER; 1.0 = linear, >1 more precision near center
    public static double YAW_EXPO = 1.8; // PLACEHOLDER
    public static double MAX_TRANSLATION = 1.0; // PLACEHOLDER
    public static double MAX_YAW = 0.85; // PLACEHOLDER; slight cap helps prevent spinouts

    public DriveController(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    /**
     * @param axial forward/backward input (positive forward)
     * @param lateral left/right input (positive right)
     * @param yaw rotation input (positive clockwise)
     * @param speedScale overall scaling 0..1
     */
    public void setDrive(double axial, double lateral, double yaw, double speedScale) {
        double a = shapeTranslation(axial);
        double l = shapeTranslation(lateral);
        double y = shapeYaw(yaw);

        setDriveShaped(a, l, y, speedScale);
    }

    /** Use this if the caller already shaped inputs. */
    public void setDriveShaped(double axial, double lateral, double yaw, double speedScale) {
        axial = clip(axial, -MAX_TRANSLATION, MAX_TRANSLATION);
        lateral = clip(lateral, -MAX_TRANSLATION, MAX_TRANSLATION);
        yaw = clip(yaw, -MAX_YAW, MAX_YAW);

        // Mecanum mixing
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        // Normalize so max magnitude <= 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Apply speed scaling
        speedScale = clip(speedScale, 0.0, 1.0);
        frontLeftPower *= speedScale;
        frontRightPower *= speedScale;
        backLeftPower *= speedScale;
        backRightPower *= speedScale;

        write(0, frontLeft, frontLeftPower);
        write(1, frontRight, frontRightPower);
        write(2, backLeft, backLeftPower);
        write(3, backRight, backRightPower);
    }

    /** Stop the drive immediately. */
    public void stop() {
        write(0, frontLeft, 0.0);
        write(1, frontRight, 0.0);
        write(2, backLeft, 0.0);
        write(3, backRight, 0.0);
    }

    /** Writes a motor's power unless it is within POWER_WRITE_EPSILON of the last one written. */
    private void write(int i, DcMotor motor, double power) {
        double last = lastPower[i];
        boolean changed = Double.isNaN(last)
                || (power == 0.0 ? last != 0.0 : Math.abs(power - last) >= POWER_WRITE_EPSILON);
        if (!changed) return;
        motor.setPower(power);
        lastPower[i] = power;
    }

    // ===== Helpers =====

    /** Deadband + expo shaping applied to a translation stick (public so callers that supply their own yaw can match it). */
    public static double shapeTranslation(double x) {
        x = applyDeadband(x, STICK_DEADBAND);
        return expo(x, TRANSLATION_EXPO);
    }

    private static double shapeYaw(double x) {
        x = applyDeadband(x, STICK_DEADBAND);
        return expo(x, YAW_EXPO);
    }

    private static double applyDeadband(double x, double deadband) {
        if (Math.abs(x) <= deadband) return 0.0;
        // Rescale to keep continuity after deadband removal
        double sign = Math.signum(x);
        double mag = (Math.abs(x) - deadband) / (1.0 - deadband);
        return sign * clip(mag, 0.0, 1.0);
    }

    private static double expo(double x, double exponent) {
        double sign = Math.signum(x);
        return sign * Math.pow(Math.abs(x), exponent);
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
