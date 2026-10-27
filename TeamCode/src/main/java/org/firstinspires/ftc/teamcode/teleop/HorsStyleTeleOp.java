package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.IntakeController;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;

import java.util.List;

/**
 * Main teleop, modeled on HORS's OfficialHORS layout but for THIS robot:
 * 4 mecanum drive motors, 2 intake motors, 2 INDEPENDENT shooters (pollen
 * and nectar, one encoder motor each, different speeds), 2 independent
 * gate servos (one per shooter). NO turret, no claw, no hood.
 *
 * Hardware map names (must match the Robot Controller config exactly):
 *   frontLeft, backLeft, frontRight, backRight (drive)
 *   intakeLeft, intakeRight (intake)
 *   shooterRight = pollen, shooterLeft = nectar (see MechanismConstants)
 *   gateRight = pollen gate, gateLeft = nectar gate (servos)
 *
 * Controls (gamepad1; gamepad2 mirrors the mechanism buttons):
 *   Left stick ......... drive (forward / strafe)
 *   Right stick X ...... turn
 *   Left bumper (hold) . slow mode (0.45x)
 *   Right trigger ...... intake IN
 *   Left trigger ....... intake OUT
 *   Dpad down .......... toggle both shooters on/off (each at its own target)
 *   Dpad left / right .. POLLEN shooter target RPM -50 / +50
 *   B .................. toggle both gates
 *   X .................. open both gates
 *   A .................. close both gates
 *
 * This keeps the seed's button map; the MainHORS control map and aim
 * system replace it in the teleop rebuild.
 *
 * The shooters start OFF and the gates start CLOSED. Shooter and gate
 * values live in MechanismConstants and are PLACEHOLDERS.
 */
@TeleOp(name = "HORS Style TeleOp", group = "Linear OpMode")
public class HorsStyleTeleOp extends LinearOpMode {

    // Drive scaling
    private static final double NORMAL_DRIVE_SCALE = 1.0;
    private static final double SLOW_DRIVE_SCALE = 0.45;

    // Rumble cooldown (so it does not buzz every loop at target).
    private static final long RUMBLE_COOLDOWN_MS = 1200;
    private long lastAtTargetRumbleMs = 0L;

    // Drive + mechanisms
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor intakeLeft, intakeRight;

    private DriveController driveController;
    private ShooterController pollenShooter;
    private ShooterController nectarShooter;
    private IntakeController intake;
    private GateController pollenGate;
    private GateController nectarGate;

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

        DcMotor pollenShooterMotor = hardwareMap.get(DcMotor.class, MechanismConstants.POLLEN_SHOOTER_NAME);
        DcMotor nectarShooterMotor = hardwareMap.get(DcMotor.class, MechanismConstants.NECTAR_SHOOTER_NAME);

        Servo pollenGateServo = hardwareMap.get(Servo.class, MechanismConstants.POLLEN_GATE_NAME);
        Servo nectarGateServo = hardwareMap.get(Servo.class, MechanismConstants.NECTAR_GATE_NAME);

        VoltageSensor batterySensor = null;
        try {
            batterySensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {
        }

        // Directions: right drive reversed. Shooter directions come from
        // MechanismConstants and still need verifying on the robot.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        pollenShooterMotor.setDirection(MechanismConstants.POLLEN_SHOOTER_DIRECTION);
        nectarShooterMotor.setDirection(MechanismConstants.NECTAR_SHOOTER_DIRECTION);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);

        // Modes: drive brakes so the robot holds still when sticks center;
        // intakes plain. ShooterController sets the shooter modes itself.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveController = new DriveController(
                frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        pollenShooter = new ShooterController(
                MechanismConstants.POLLEN_SHOOTER, pollenShooterMotor, telemetry, batterySensor);
        nectarShooter = new ShooterController(
                MechanismConstants.NECTAR_SHOOTER, nectarShooterMotor, telemetry, batterySensor);
        pollenShooter.setShooterOn(false); // starts OFF; dpad down spins up
        nectarShooter.setShooterOn(false);
        intake = new IntakeController(intakeLeft, intakeRight);
        pollenGate = new GateController("Pollen", pollenGateServo,
                MechanismConstants.POLLEN_GATE_OPEN, MechanismConstants.POLLEN_GATE_CLOSED);
        nectarGate = new GateController("Nectar", nectarGateServo,
                MechanismConstants.NECTAR_GATE_OPEN, MechanismConstants.NECTAR_GATE_CLOSED);

        telemetry.addData("Status", "Initialized -- shooters OFF, gates CLOSED");
        telemetry.addData("Drive", "Slow mode: left bumper");
        telemetry.addData("Shooters", "Dpad down = on/off, dpad left/right = POLLEN RPM -/+50");
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

            // ----- Shooters: dpad down toggles both, dpad left/right trims pollen -----
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                boolean on = !pollenShooter.isShooterOn();
                pollenShooter.setShooterOn(on);
                nectarShooter.setShooterOn(on);
            }
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) pollenShooter.adjustTargetRpm(-50.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) pollenShooter.adjustTargetRpm(50.0);
            dpadRightLast = dpadRightNow;

            pollenShooter.update();
            nectarShooter.update();

            // Rumble once (with cooldown) when either shooter first hits target.
            // Read both flags every loop so neither stays latched.
            boolean pollenReached = pollenShooter.justReachedTarget();
            boolean nectarReached = nectarShooter.justReachedTarget();
            if ((pollenReached || nectarReached)
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

            // ----- Gates: B toggles, X opens, A closes (both gates for now) -----
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast) {
                pollenGate.toggle();
                nectarGate.toggle();
            }
            bPressedLast = bNow;

            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                pollenGate.setClosed(false);
                nectarGate.setClosed(false);
            }
            xPressedLast = xNow;

            boolean aNow = gamepad1.a || gamepad2.a;
            if (aNow && !aPressedLast) {
                pollenGate.setClosed(true);
                nectarGate.setClosed(true);
            }
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
            addShooterTelemetry(pollenShooter);
            addShooterTelemetry(nectarShooter);
            telemetry.addData("Gates", "pollen=%s nectar=%s",
                    pollenGate.isClosed() ? "CLOSED" : "OPEN",
                    nectarGate.isClosed() ? "CLOSED" : "OPEN");
            telemetry.addData("FL/FR/BL/BR",
                    "%.2f / %.2f / %.2f / %.2f",
                    frontLeftDrive.getPower(), frontRightDrive.getPower(),
                    backLeftDrive.getPower(), backRightDrive.getPower());
            telemetry.update();
        }

        driveController.stop();
        intake.stop();
        pollenShooter.setShooterOn(false);
        nectarShooter.setShooterOn(false);
        pollenShooter.update();
        nectarShooter.update();
    }

    private void addShooterTelemetry(ShooterController shooter) {
        telemetry.addData(shooter.getLabel() + " shooter",
                "on=%s target=%.0f rpm current=%.0f rpm atTarget=%s",
                shooter.isShooterOn(), shooter.getTargetRpm(),
                shooter.getCurrentRpm(), shooter.isAtTarget());
    }
}
