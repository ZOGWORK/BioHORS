package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Plain autonomous: drive forward a short distance, then stop and park.
 *
 * This auto does NOT use PedroPathing on purpose -- it is just move-and-park.
 * The project is still the PedroPathing Quickstart, so when you want real
 * path autos later, build them on the follower in pedro/Constants.
 *
 * How it works: all four drive motors run to encoder targets
 * (RUN_TO_POSITION), then hold with BRAKE so the robot parks in place.
 *
 * ============================================================================
 * TODO: CHANGE THESE. The numbers below are small random placeholders so
 * the auto runs at all -- they are NOT tuned for any field position.
 * Drive the robot, measure how far it actually goes, and set DRIVE_TICKS
 * to the distance you want. Same for DRIVE_POWER and TIMEOUT_SECONDS.
 * ============================================================================
 */
@Autonomous(name = "Simple Park Auto", group = "Linear OpMode")
public class SimpleParkAuto extends LinearOpMode {

    // TODO: CHANGE THIS -- encoder ticks to drive forward. Small placeholder.
    // (Ticks are per-motor encoder counts; the number for a given distance
    // depends on your drive motors' ticks/rev and your wheel size.)
    private static final int DRIVE_TICKS = 1000;

    // TODO: CHANGE THIS -- drive power for the park move (0..1).
    private static final double DRIVE_POWER = 0.5;

    // TODO: CHANGE THIS -- give up after this many seconds even if the
    // motors have not reached their targets (so auto never hangs).
    private static final double TIMEOUT_SECONDS = 3.0;

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        // Same directions as the teleop. NOTE: with a motor set to REVERSE,
        // the SDK flips its encoder sign too, so a positive target still
        // means "forward" for every wheel here.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders, then run to position.
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition(DRIVE_TICKS);
        backLeftDrive.setTargetPosition(DRIVE_TICKS);
        frontRightDrive.setTargetPosition(DRIVE_TICKS);
        backRightDrive.setTargetPosition(DRIVE_TICKS);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Ready -- will drive forward %d ticks then park", DRIVE_TICKS);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        frontLeftDrive.setPower(DRIVE_POWER);
        backLeftDrive.setPower(DRIVE_POWER);
        frontRightDrive.setPower(DRIVE_POWER);
        backRightDrive.setPower(DRIVE_POWER);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()
                && timer.seconds() < TIMEOUT_SECONDS
                && (frontLeftDrive.isBusy() || backLeftDrive.isBusy()
                || frontRightDrive.isBusy() || backRightDrive.isBusy())) {
            telemetry.addData("Driving to park", "FL %d / FR %d / BL %d / BR %d",
                    frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(),
                    backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // Park: cut power and let BRAKE hold the robot in place.
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        telemetry.addData("Status", "Parked");
        telemetry.update();
    }
}
