package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlowerBarController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.IntakeController;
import org.firstinspires.ftc.teamcode.subsystems.IntakeJamGuard;
import org.firstinspires.ftc.teamcode.subsystems.IntakeLoadSensor;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;

/**
 * Owns this robot's hardware and builds every subsystem from one HardwareMap. The teleop, the
 * Zenith auto robot and the SimLoop tests all construct one of these, so the hardware names,
 * motor directions and modes are written down exactly once.
 *
 * It does NOT build a localizer or a Pedro follower: the teleop and the auto each own one
 * (see pedro/Constants), so two objects never read the Pinpoint in the same loop.
 */
public class BioHorsRobot {

    public final DcMotor frontLeft;
    public final DcMotor backLeft;
    public final DcMotor frontRight;
    public final DcMotor backRight;

    public final DriveController drive;
    public final IntakeController intake;
    /** Pollen count from the intake's encoders (both intake motors). */
    public final IntakeLoadSensor intakeLoad;
    /** Teleop jam clearing: reverses a stalled intake briefly (both intake motors' encoders). */
    public final IntakeJamGuard intakeJam;
    public final ShooterController pollenShooter;
    public final ShooterController nectarShooter;
    public final GateController pollenGate;
    public final GateController nectarGate;
    public final FlowerBarController flowerBar;

    /** The battery voltage sensor, for the low-battery warning. Null if the config has none. */
    private final VoltageSensor battery;

    /** The Pinpoint, for its device status. Null if the config has none. */
    public final GoBildaPinpointDriver pinpoint;

    public BioHorsRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Bulk read: one read per hub per loop instead of per device.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        DcMotorEx intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        DcMotorEx intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        DcMotor pollenShooterMotor = hardwareMap.get(DcMotor.class, MechanismConstants.POLLEN_SHOOTER_NAME); //shooterRight
        DcMotor nectarShooterMotor = hardwareMap.get(DcMotor.class, MechanismConstants.NECTAR_SHOOTER_NAME); //shooterLeft
        Servo pollenGateServo = hardwareMap.get(Servo.class, MechanismConstants.POLLEN_GATE_NAME); //gateRight
        Servo nectarGateServo = hardwareMap.get(Servo.class, MechanismConstants.NECTAR_GATE_NAME); //gateLeft
        Servo flowerLeftServo = hardwareMap.get(Servo.class, MechanismConstants.FLOWER_BAR_LEFT_NAME); //flowerLeft
        Servo flowerRightServo = hardwareMap.get(Servo.class, MechanismConstants.FLOWER_BAR_RIGHT_NAME); //flowerRight

        GoBildaPinpointDriver pp = null;
        try {
            pp = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (Exception ignored) {
        }
        pinpoint = pp;

        VoltageSensor batterySensor = null;
        try {
            batterySensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {
        }

        battery = batterySensor;

        // Directions: left drive reversed (same as pedro/Constants; all four flipped after testing
        // on the robot). Shooter directions come from MechanismConstants and still need verifying.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        pollenShooterMotor.setDirection(MechanismConstants.POLLEN_SHOOTER_DIRECTION);
        nectarShooterMotor.setDirection(MechanismConstants.NECTAR_SHOOTER_DIRECTION);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);

        // Drive brakes so the robot holds still when sticks center; intakes plain.
        // ShooterController sets the shooter modes itself.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive = new DriveController(frontLeft, frontRight, backLeft, backRight);
        intake = new IntakeController(intakeLeft, intakeRight);
        intakeLoad = new IntakeLoadSensor(intakeLeft, intakeRight);
        intakeJam = new IntakeJamGuard(intakeLeft, intakeRight);
        pollenShooter = new ShooterController(
                MechanismConstants.POLLEN_SHOOTER, pollenShooterMotor, telemetry, batterySensor);
        nectarShooter = new ShooterController(
                MechanismConstants.NECTAR_SHOOTER, nectarShooterMotor, telemetry, batterySensor);
        pollenGate = new GateController("Pollen", pollenGateServo,
                MechanismConstants.POLLEN_GATE_OPEN, MechanismConstants.POLLEN_GATE_CLOSED);
        nectarGate = new GateController("Nectar", nectarGateServo,
                MechanismConstants.NECTAR_GATE_OPEN, MechanismConstants.NECTAR_GATE_CLOSED);
        flowerBar = new FlowerBarController(flowerLeftServo, flowerRightServo,
                MechanismConstants.FLOWER_BAR_STOWED, MechanismConstants.FLOWER_BAR_DOWN,
                MechanismConstants.FLOWER_BAR_SHOOTING);
    }

    /** Battery volts, or NaN if there is no voltage sensor or it fails. */
    public double batteryVolts() {
        if (battery == null) return Double.NaN;
        try {
            return battery.getVoltage();
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    /** True when the Pinpoint reports READY. False if it is missing, calibrating or faulted. */
    public boolean pinpointReady() {
        if (pinpoint == null) return false;
        try {
            return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        } catch (Exception e) {
            return false;
        }
    }

    /** Run once per loop: both shooters' velocity loops, and the intake's pollen count. */
    public void updateMechanisms() {
        intakeLoad.update(intake.getPower());
        pollenShooter.update();
        nectarShooter.update();
    }

    /** Everything off, gates shut and the bar stowed (no pollen rolls out after the match). Call
     *  when the OpMode stops. */
    public void stop() {
        drive.stop();
        intake.stop();
        pollenGate.setClosed(true);
        nectarGate.setClosed(true);
        flowerBar.setShooting(false);
        flowerBar.setDown(false);
        pollenShooter.setShooterOn(false);
        nectarShooter.setShooterOn(false);
        pollenShooter.update();
        nectarShooter.update();
    }
}
