package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.aim.AimController;
import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.firstinspires.ftc.teamcode.aim.FieldTargets;
import org.firstinspires.ftc.teamcode.aim.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;

/**
 * The MainHORS teleop: the MainHORS control scheme on this robot, plus odometry aim.
 *
 * Controls (gamepad1; gamepad2 mirrors the mechanism buttons):
 *   Left stick ............ drive (forward / strafe)
 *   Right stick X ......... turn (overridden while aiming)
 *   Left trigger (hold) ... AIM: turn the robot to face our hive's cell on this half of the field
 *   Right bumper (hold) ... slow mode (0.45x)
 *   Right trigger ......... intake in
 *   Y (hold) .............. outtake
 *   Dpad down ............. toggle both shooters on/off (each at its own target)
 *   Dpad left / right ..... POLLEN shooter target RPM - / + (MechanismConstants.POLLEN_TRIM_STEP_RPM)
 *   Left bumper / A ....... NECTAR shooter target RPM - / + (MechanismConstants.NECTAR_TRIM_STEP_RPM)
 *   B ..................... toggle both gates (the flower bar lifts slightly while they are open)
 *   X ..................... toggle the flower bar: DOWN (collect from a flower) / STOWED
 *
 * Before PLAY: dpad left = RED, dpad right = BLUE. If an auto ran first, its alliance and last
 * pose are picked up from the blackboard.
 *
 * Aim uses the Pinpoint pose and the hive's fixed field coordinates; there is no camera. The
 * target cell follows the robot's half of the field (north cell when y > 0, south when y < 0),
 * because a launch only scores from the up cell's own quarter. If the
 * Pinpoint is not READY or the pose is not a number, aiming falls back to manual turn and
 * telemetry warns. Drive and strafe stay with the driver while aiming.
 *
 * Shooters start OFF and gates CLOSED. Tuning values are placeholders (MechanismConstants,
 * AimController).
 */
@TeleOp(name = "MainHORS TeleOp", group = "Linear OpMode")
public class MainHorsTeleOp extends LinearOpMode {

    private static final double NORMAL_DRIVE_SCALE = 1.0;
    private static final double SLOW_DRIVE_SCALE = 0.45;
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double RUMBLE_COOLDOWN_S = 1.2;

    // Where the robot is assumed to start when no auto ran before this teleop. RED values; BLUE
    // is the point mirror. The auto's `start` waypoint: back on the south wall (the drive centre
    // 7.8 in off it, MEASURED FROM CAD), facing the hive. Not measured on a real field.
    public static double DEFAULT_START_X_IN = -15.0;   // NEEDS MEASUREMENT (= autos/waypoints.json start)
    public static double DEFAULT_START_Y_IN = -64.2;   // MEASURED FROM CAD (-72 + 7.8)
    public static double DEFAULT_START_HEADING_RAD = Math.PI / 2.0; // facing the hive (north)

    private BioHorsRobot robot;
    private Localizer localizer;
    private final AimController aim = new AimController();

    private Alliance alliance = Alliance.RED;
    private FieldTargets.Cell targetCell = FieldTargets.upCellAtKickoff(Alliance.RED);
    private boolean poseFromAuto = false;

    // Edge-detect state (a press fires once, not every loop).
    private boolean dpadDownLast, dpadLeftLast, dpadRightLast, bLast, xLast, aLast, lbLast;

    @Override
    public void runOpMode() {
        robot = new BioHorsRobot(hardwareMap, telemetry);
        localizer = Constants.createLocalizer(hardwareMap);

        Alliance saved = PoseHandoff.savedAlliance();
        double[] savedPose = PoseHandoff.savedPose();
        if (saved != null) alliance = saved;
        poseFromAuto = savedPose != null;
        if (poseFromAuto) {
            localizer.setPose(new Pose(savedPose[0], savedPose[1], savedPose[2]));
        } else {
            localizer.setPose(defaultStartPose(alliance));
        }
        targetCell = FieldTargets.upCellAtKickoff(alliance);

        // ----- INIT: pick the alliance -----
        while (!isStarted() && !isStopRequested()) {
            Alliance picked = null;
            if (gamepad1.dpad_left) picked = Alliance.RED;
            if (gamepad1.dpad_right) picked = Alliance.BLUE;
            if (picked != null && picked != alliance) {
                alliance = picked;
                targetCell = FieldTargets.upCellAtKickoff(alliance);
                if (!poseFromAuto) localizer.setPose(defaultStartPose(alliance));
            }
            // Track the mechanism buttons during INIT too, so a button still held at PLAY (the
            // dpad from picking the alliance, say) is not read as a fresh press on the first loop.
            seedEdgeDetectors();
            localizer.update();
            targetCell = FieldTargets.cellForHalf(localizer.pose().y(), targetCell);
            telemetry.addData("Alliance", "%s  (dpad left = RED, dpad right = BLUE)", alliance);
            telemetry.addData("Start pose", poseFromAuto ? "from auto" : "default (unmeasured)");
            addPoseTelemetry();
            telemetry.addData("Status", "Initialized -- shooters OFF, gates CLOSED");
            telemetry.update();
        }
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        ElapsedTime loopTimer = new ElapsedTime();
        Pose lastPose = localizer.pose();
        ElapsedTime sinceRumble = new ElapsedTime();
        sinceRumble.reset();
        boolean rumbledOnce = false;

        while (opModeIsActive()) {
            localizer.update();
            Pose pose = localizer.pose();
            boolean poseOk = poseUsable(pose);

            // Field velocity from the pose change, for the aim feedforward.
            double dt = loopTimer.seconds();
            loopTimer.reset();
            double vx = 0.0;
            double vy = 0.0;
            if (poseOk && lastPose != null && dt > 1e-4 && dt < 0.25
                    && Double.isFinite(lastPose.x()) && Double.isFinite(lastPose.y())) {
                vx = (pose.x() - lastPose.x()) / dt;
                vy = (pose.y() - lastPose.y()) / dt;
            }
            lastPose = pose;

            // ----- Target: the cell of our hive on the robot's half of the field -----
            if (poseOk) targetCell = FieldTargets.cellForHalf(pose.y(), targetCell);

            // ----- Drive, with aim overriding manual turn while left trigger is held -----
            double scale = gamepad1.right_bumper ? SLOW_DRIVE_SCALE : NORMAL_DRIVE_SCALE;
            double axial = DriveController.shapeTranslation(-gamepad1.left_stick_y);
            double lateral = DriveController.shapeTranslation(gamepad1.left_stick_x);
            boolean aimHeld = gamepad1.left_trigger > TRIGGER_THRESHOLD;
            boolean aiming = aimHeld && poseOk;
            if (aiming) {
                double[] t = FieldTargets.cellAimPoint(alliance, targetCell);
                double yaw = aim.yawCommand(pose.x(), pose.y(), pose.heading(), vx, vy, t[0], t[1]);
                // Yaw has priority: translation gets only the headroom the aim yaw leaves, so the
                // mecanum normalization never shrinks the yaw while the driver strafes hard. The
                // aim yaw is not slowed by slow mode: it is already capped.
                double a = axial * scale;
                double l = lateral * scale;
                double room = 1.0 - Math.abs(yaw);
                double used = Math.abs(a) + Math.abs(l);
                if (used > room && used > 1e-9) {
                    a *= room / used;
                    l *= room / used;
                }
                robot.drive.setDriveShaped(a, l, yaw, 1.0);
            } else {
                robot.drive.setDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                        gamepad1.right_stick_x, scale);
            }

            // ----- Shooters: dpad down toggles both, dpad left/right trims pollen -----
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                boolean on = !robot.pollenShooter.isShooterOn();
                robot.pollenShooter.setShooterOn(on);
                robot.nectarShooter.setShooterOn(on);
            }
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) robot.pollenShooter.adjustTargetRpm(-MechanismConstants.POLLEN_TRIM_STEP_RPM);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) robot.pollenShooter.adjustTargetRpm(MechanismConstants.POLLEN_TRIM_STEP_RPM);
            dpadRightLast = dpadRightNow;

            // Nectar has its own trim and step: left bumper down, A up.
            boolean lbNow = gamepad1.left_bumper || gamepad2.left_bumper;
            if (lbNow && !lbLast) robot.nectarShooter.adjustTargetRpm(-MechanismConstants.NECTAR_TRIM_STEP_RPM);
            lbLast = lbNow;

            boolean aNow = gamepad1.a || gamepad2.a;
            if (aNow && !aLast) robot.nectarShooter.adjustTargetRpm(MechanismConstants.NECTAR_TRIM_STEP_RPM);
            aLast = aNow;

            robot.updateMechanisms();

            // Rumble once (with cooldown) when either shooter first reaches its target.
            // Read both flags every loop so neither stays latched.
            boolean pollenReached = robot.pollenShooter.justReachedTarget();
            boolean nectarReached = robot.nectarShooter.justReachedTarget();
            if ((pollenReached || nectarReached)
                    && (!rumbledOnce || sinceRumble.seconds() >= RUMBLE_COOLDOWN_S)) {
                try {
                    gamepad1.rumble(220);
                    gamepad2.rumble(220);
                } catch (Throwable ignored) {
                }
                sinceRumble.reset();
                rumbledOnce = true;
            }

            // ----- Gates: B toggles both -----
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bLast) {
                boolean close = !robot.pollenGate.isClosed();
                robot.pollenGate.setClosed(close);
                robot.nectarGate.setClosed(close);
            }
            bLast = bNow;

            // ----- Flower bar: X toggles down / stowed; lifted slightly while the gates are open -----
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xLast) robot.flowerBar.toggle();
            xLast = xNow;
            robot.flowerBar.setShooting(!robot.pollenGate.isClosed() || !robot.nectarGate.isClosed());

            // ----- Intake: right trigger in, Y out -----
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD || gamepad2.right_trigger > TRIGGER_THRESHOLD) {
                robot.intake.intake();
            } else if (gamepad1.y || gamepad2.y) {
                robot.intake.outtake();
            } else {
                robot.intake.stop();
            }

            // ----- Telemetry -----
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Flower bar", robot.flowerBar.position());
            telemetry.addData("Target", "%s hive, %s cell", alliance, targetCell);
            if (aimHeld && !poseOk) {
                telemetry.addData("Aiming", "OFF -- POSE INVALID, manual turn");
            } else {
                telemetry.addData("Aiming", aiming ? "ON" : "off");
            }
            telemetry.addData("Heading error", "%.1f deg", Math.toDegrees(headingErrorTo(pose)));
            addPoseTelemetry();
            telemetry.addData("DriveScale", "%.2f", scale);
            addShooterTelemetry("Pollen", robot.pollenShooter);
            addShooterTelemetry("Nectar", robot.nectarShooter);
            telemetry.addData("Gates", "pollen=%s nectar=%s",
                    robot.pollenGate.isClosed() ? "CLOSED" : "OPEN",
                    robot.nectarGate.isClosed() ? "CLOSED" : "OPEN");
            telemetry.update();
        }

        robot.stop();
    }

    /** Sets every edge detector to the buttons' current state. */
    private void seedEdgeDetectors() {
        dpadDownLast = gamepad1.dpad_down || gamepad2.dpad_down;
        dpadLeftLast = gamepad1.dpad_left || gamepad2.dpad_left;
        dpadRightLast = gamepad1.dpad_right || gamepad2.dpad_right;
        xLast = gamepad1.x || gamepad2.x;
        aLast = gamepad1.a || gamepad2.a;
        bLast = gamepad1.b || gamepad2.b;
        lbLast = gamepad1.left_bumper || gamepad2.left_bumper;
    }

    /** The default start pose for an alliance: the RED value, point-mirrored for BLUE. */
    static Pose defaultStartPose(Alliance alliance) {
        if (alliance == Alliance.RED) {
            return new Pose(DEFAULT_START_X_IN, DEFAULT_START_Y_IN, DEFAULT_START_HEADING_RAD);
        }
        return new Pose(-DEFAULT_START_X_IN, -DEFAULT_START_Y_IN,
                AimController.wrap(DEFAULT_START_HEADING_RAD + Math.PI));
    }

    /** A pose may be used for aiming only when the Pinpoint is READY and the numbers are real. */
    private boolean poseUsable(Pose pose) {
        return robot.pinpointReady() && pose != null
                && Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }

    private double headingErrorTo(Pose pose) {
        if (pose == null) return 0.0;
        double[] t = FieldTargets.cellAimPoint(alliance, targetCell);
        return AimController.wrap(
                AimController.desiredHeading(pose.x(), pose.y(), t[0], t[1]) - pose.heading());
    }

    private void addPoseTelemetry() {
        Pose p = localizer.pose();
        boolean ok = poseUsable(p);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f deg %s",
                p.x(), p.y(), Math.toDegrees(p.heading()), ok ? "" : "(INVALID)");
    }

    private void addShooterTelemetry(String label, org.firstinspires.ftc.teamcode.subsystems.ShooterController s) {
        telemetry.addData(label + " shooter", "on=%s target=%.0f rpm current=%.0f rpm atTarget=%s",
                s.isShooterOn(), s.getTargetRpm(), s.getCurrentRpm(), s.isAtTarget());
    }
}
