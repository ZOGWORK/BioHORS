package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * PedroPathing v3 setup for this robot.
 *
 * The Zenith auto drives with the follower built here. The MainHORS teleop
 * uses only the localizer (for aim); it drives the motors itself.
 *
 * Localizer: goBILDA Pinpoint odometry computer with TWO odometry pods
 * (one forward / X pod, one strafe / Y pod), the same setup the team runs.
 *
 * ============================================================================
 * TODO: MEASURE / VERIFY THE PINPOINT ITEMS BELOW. Pod offsets, pod type,
 * and pod directions are robot-specific and cannot be guessed:
 *  - Run the Quickstart's Pinpoint Tuner (Tuning OpMode) on the robot. It
 *    finds the pod directions and the X/Y offsets for you.
 *  - Pod type: set to goBILDA_4_BAR_POD below. If your pods are the
 *    swingarm style instead, change it to goBILDA_SWINGARM_POD.
 *  - Offsets are in INCHES, measured from the Pinpoint to each pod's
 *    contact point along the robot's axes (the tuner reports them).
 * ============================================================================
 */
public class Constants {

    // ---- Pinpoint localizer config (2 odometry pods) ----
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        // TODO: confirm pod type -- 4-bar pods here; swingarm pods use
        // GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD.
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // TODO: MEASURE THESE with the Pinpoint Tuner (inches).
        // MEASURED FROM CAD (33950 Robot.step): the X (forward) pod sits 2.84 in to the right of
        // the drive centre (5.1 in behind it). goBILDA's convention is left-positive, so -2.84.
        // VERIFY sign and value with the Pinpoint Tuner.
        c.xPodOffset.set(-2.84);
        // NEEDS MEASUREMENT: the Y (strafe) pod is not in the CAD.
        c.yPodOffset.set(0.0); // PLACEHOLDER
        // TODO: VERIFY with the Pinpoint Tuner -- flip to REVERSED if the
        // pose drives backwards on either axis.
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // PLACEHOLDER
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // PLACEHOLDER
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ---- Mecanum drivetrain config (same motors + directions as teleop) ----
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    /** Pinpoint localizer: 2 odometry pods + IMU heading. */
    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    /** Mecanum drivetrain for the follower. */
    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    // ---- Foresight (path following) ----
    // Every value below is a PLACEHOLDER until the Foresight Tuner runs on the robot (Tuning
    // OpMode -> Foresight Tuner); it prints these same fields. The values were SET FROM SIM:
    // chosen so the follower drives the SimLoop model (TeamCode/src/test/.../sim/BioHorsSim)
    // cleanly, which says nothing about the real robot.
    // Keep MAX_FORWARD/STRAFE and the decelerations equal to autos/robot.json's kinematics.
    public static double MAX_FORWARD_IN_PER_S = 60.0;          // PLACEHOLDER (SET FROM SIM)
    public static double MAX_STRAFE_IN_PER_S = 50.0;           // PLACEHOLDER (SET FROM SIM)
    public static double FORWARD_DECEL_IN_PER_S2 = 60.0;       // PLACEHOLDER (SET FROM SIM)
    public static double STRAFE_DECEL_IN_PER_S2 = 50.0;        // PLACEHOLDER (SET FROM SIM)

    /** A fresh algorithm config (each follower owns one, since a path's speed cap edits it). */
    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.04))
                    .put(2.5, Controller.proportional(0.12)));            // PLACEHOLDER (SET FROM SIM)
            c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.06))
                    .put(2.5, Controller.proportional(0.15)));            // PLACEHOLDER (SET FROM SIM)
            c.coast.set(Controller.proportionalFeedforward(0.02));        // PLACEHOLDER (SET FROM SIM)
            c.brake.set(Controller.proportionalFeedforward(0.05));        // PLACEHOLDER (SET FROM SIM)
            c.headingFeedback.set(Controller.proportional(3.0));          // PLACEHOLDER (SET FROM SIM)
            // A path ends only once the heading is within 3 deg (Pedro's default is 11 deg), so a
            // volley that follows a path starts facing the cell. PLACEHOLDER (SET FROM SIM).
            c.headingDeviationTolerance.set(Math.toRadians(3.0));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06, 0.004));      // PLACEHOLDER
            c.linearBrakeCoefficients.set(Matrix.diag(0.08, 0.1));              // PLACEHOLDER
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015, 0.002));       // PLACEHOLDER
            c.maxAchievableForwardVelocity.set(MAX_FORWARD_IN_PER_S);
            c.maxAchievableStrafeVelocity.set(MAX_STRAFE_IN_PER_S);
            c.naturalForwardDeceleration.set(FORWARD_DECEL_IN_PER_S2);
            c.naturalStrafeDeceleration.set(STRAFE_DECEL_IN_PER_S2);
        });
    }

    /**
     * Follower for path autos (the Zenith auto): Pinpoint localizer, mecanum drivetrain and
     * Foresight. The teleop does not use it; it reads the localizer directly.
     */
    public static Follower createFollower(HardwareMap hardwareMap, ForesightConfig config) {
        return new Follower(
                createLocalizer(hardwareMap),
                createDrivetrain(hardwareMap),
                new Foresight(config));
    }

    /** Follower with a fresh {@link #foresightConfig()}. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return createFollower(hardwareMap, foresightConfig());
    }
}
