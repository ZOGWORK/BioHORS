package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * PedroPathing v3 setup for this robot, ready for LATER autos.
 *
 * The teleop (HorsStyleTeleOp) and the simple park auto do NOT use this
 * follower yet -- they drive the motors directly on purpose. This file
 * exists so that when you want real PedroPathing path autos, the follower,
 * drivetrain, and Pinpoint localizer are already wired up and you only
 * have to write the path.
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
        // TODO: MEASURE THESE with the Pinpoint Tuner (inches). Placeholders.
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        // TODO: VERIFY with the Pinpoint Tuner -- flip to REVERSED if the
        // pose drives backwards on either axis.
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ---- Mecanum drivetrain config (same motors + directions as teleop) ----
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    /** Pinpoint localizer: 2 odometry pods + IMU heading. */
    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    /** Mecanum drivetrain for the follower. */
    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    /**
     * Follower for future PedroPathing autos.
     *
     * The third argument (Foresight / algorithm) is null for now: pose
     * tracking works without it, and you add a Foresight when you build
     * your first path auto. This follower is NOT used by the teleop or
     * the simple park auto yet -- it is here so the wiring is done.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                createLocalizer(hardwareMap),
                createDrivetrain(hardwareMap),
                null);
    }
}
