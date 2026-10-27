package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.IntakeController;

import java.util.List;

/**
 * Main teleop, modeled on HORS's OfficialHORS layout but for THIS robot:
 * 4 mecanum drive motors, 2 intake motors, 2 encoder flywheel (shooter)
 * motors, 2 gate servos. NO turret, no claw, no hood.
 *
 * Hardware map names (must match the Robot Controller config exactly):
 *   frontLeft, backLeft, frontRight, backRight (drive)
 *   intakeLeft, intakeRight (intake)
 *   shooterLeft, shooterRight (flywheel, with encoders)
 *   gateLeft, gateRight (servos)
 *
 * Controls (gamepad1; gamepad2 mirrors the mechanism buttons):
 *   Left stick ......... drive (forward / strafe)
 *   Right stick X ...... turn
 *   Left bumper (hold) . slow mode (0.45x)
 *   Right trigger ...... intake IN
 *   Left trigger ....... intake OUT
 *   Dpad down .......... toggle shooter on/off
 *   Dpad left / right .. shooter target RPM -50 / +50
 *   B .................. toggle gates
 *   X .................. open gates
 *   A .................. close gates
 *
 * The shooter starts OFF. Press dpad down once to spin it up.
 * Flywheel PIDF values live in FlywheelController and are PLACEHOLDERS --
 * tune them on this robot before a match.
 */
@TeleOp(name = "HORS Style TeleOp", group = "Linear OpMode")
public class HorsStyleTeleOp extends LinearOpMode {

    // Drive scaling
    private static final double NORMAL_DRIVE_SCALE = 1.0;
    private static final double SLOW_DRIVE_SCALE = 0.45;

    // TODO: TUNE THESE -- placeholder gate positions. Read the real open /
    // closed positions off the driver station and put them here.
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.485;

    // Rumble cooldown (so it does not buzz every loop at target).
    private static final long RUMBLE_COOLDOWN_MS = 1200;
    private long lastAtTargetRumbleMs = 0L;

    // Drive + mechanisms
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor intakeLeft, intakeRight;
    private DcMotor shooterLeft, shooterRight;
    private Servo gateLeft, gateRight;

    private DriveController driveController;
    private FlywheelController flywheel;
    private IntakeController intake;
    private GateController gateController;

    // Edge-detect state (so a button press fires once, not every loop).
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean bPressedLast = false;
    private boolean xPressedLast = false;
    private boolean aPressedLast = false;

    @Override
    public void runOpMode() {

        // Bulk read: one read per hub per loop instead of per device.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Hardware map
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");

        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        gateLeft = hardwareMap.get(Servo.class, "gateLeft");
        gateRight = hardwareMap.get(Servo.class, "gateRight");

        VoltageSensor batterySensor = null;
        try {
            batterySensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {
        }

        // Directions (same pattern as HORS: right drive reversed, one
        // shooter reversed so the pair spins opposite).
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);

        // Modes: drive brakes so the robot holds still when sticks center;
        // shooters use their encoders for velocity control; intakes plain.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveController = new DriveController(
                frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new FlywheelController(shooterLeft, shooterRight, telemetry, batterySensor);
        flywheel.setShooterOn(false); // starts OFF; dpad down spins it up
        intake = new IntakeController(intakeLeft, intakeRight);
        gateController = new GateController(gateLeft, gateRight, GATE_OPEN, GATE_CLOSED);

        telemetry.addData("Status", "Initialized -- shooter OFF, gates CLOSED");
        telemetry.addData("Drive", "Slow mode: left bumper");
        telemetry.addData("Shooter", "Dpad down = on/off, dpad left/right = RPM -/+50");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            // ----- Drive (slow mode while left bumper is held) -----
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double scale = gamepad1.left_bumper ? SLOW_DRIVE_SCALE : NORMAL_DRIVE_SCALE;
            driveController.setDrive(axial, lateral, yaw, scale);

            // ----- Flywheel: dpad down toggles, dpad left/right trims RPM -----
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) flywheel.adjustTargetRpm(-50.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) flywheel.adjustTargetRpm(50.0);
            dpadRightLast = dpadRightNow;

            flywheel.update();

            // Rumble once (with cooldown) when the flywheel first hits target.
            if (flywheel.justReachedTarget()
                    && (nowMs - lastAtTargetRumbleMs >= RUMBLE_COOLDOWN_MS)) {
                try {
                    gamepad1.rumble(220);
                } catch (Throwable ignored) {
                }
                try {
                    gamepad2.rumble(220);
                } catch (Throwable ignored) {
                }
                lastAtTargetRumbleMs = nowMs;
            }

            // ----- Gates: B toggles, X opens, A closes -----
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast) gateController.toggleGate();
            bPressedLast = bNow;

            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) gateController.setGateClosed(false);
            xPressedLast = xNow;

            boolean aNow = gamepad1.a || gamepad2.a;
            if (aNow && !aPressedLast) gateController.setGateClosed(true);
            aPressedLast = aNow;

            // ----- Intake: right trigger in, left trigger out -----
            if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                intake.intake();
            } else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                intake.outtake();
            } else {
                intake.stop();
            }

            // ----- Telemetry -----
            telemetry.addData("DriveScale", "%.2f", scale);
            telemetry.addData("Shooter", "on=%s target=%.0f rpm current=%.0f rpm atTarget=%s",
                    flywheel.isShooterOn(), flywheel.getTargetRpm(),
                    flywheel.getCurrentRpm(), flywheel.isAtTarget());
            telemetry.addData("Gates", gateController.isGateClosed() ? "CLOSED" : "OPEN");
            telemetry.addData("FL/FR/BL/BR",
                    "%.2f / %.2f / %.2f / %.2f",
                    frontLeftDrive.getPower(), frontRightDrive.getPower(),
                    backLeftDrive.getPower(), backRightDrive.getPower());
            telemetry.update();
        }

        driveController.stop();
        intake.stop();
        flywheel.setShooterOn(false);
        flywheel.update();
    }
}
