package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.PositionalServoSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.fakehardware.FakeGoBildaPinpoint;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.horizon36596.simloop.plant.PositionalServoPlant;

/**
 * This robot in SimLoop: a fake device under every name the robot code looks up (BioHorsRobot,
 * pedro/Constants), a mecanum plant behind the four drive motors, a fake Pinpoint fed from that
 * plant, the two shooter motors, and the two gate servos with a servo plant behind each.
 *
 * The robot code never sees this class. The OpMode builds BioHorsRobot and Pedro's localizer from
 * {@link #hardwareMap} exactly as it would from the Control Hub's.
 *
 * Frame: the plant's pose is the robot's TRUE pose in the field frame the code uses (inches,
 * origin at the field centre, heading radians CCW from +X; see aim/FieldTargets). The Pinpoint only
 * ever sees the plant's motion, so its pose is what the code set at init plus that motion.
 *
 * Tick order: OpModeRunner runs the OpMode's loop, then updates every fake device, then calls
 * {@link #step}. So the Pinpoint reports the chassis one tick late, as a real loop reading a
 * sensor before commanding does.
 *
 * NUMBERS: nothing here is measured. Every value is labelled. A sim built on placeholders checks
 * the code's logic and wiring, not the real robot's timing. See docs/TUNING.md.
 */
public final class BioHorsSim {

    private static final double MM_PER_IN = 25.4;

    // ----- Drive (PLACEHOLDER: motor and wheel are not confirmed; a goBILDA 312 rpm motor,
    // 537.7 counts/rev, on a 104 mm mecanum wheel is assumed) -----
    static final double DRIVE_TICKS_PER_REV = 537.7;            // PLACEHOLDER (312 rpm motor SPEC)
    static final double WHEEL_RADIUS_IN = 104.0 / 2.0 / MM_PER_IN; // MEASURED FROM CAD: 104 mm goBILDA mecanum
    static final double DRIVE_TICKS_PER_INCH = DRIVE_TICKS_PER_REV / (2.0 * Math.PI * WHEEL_RADIUS_IN);
    static final double DRIVE_MAX_TICKS_PER_S = 312.0 * DRIVE_TICKS_PER_REV / 60.0; // PLACEHOLDER (free speed)
    /** How fast a drive motor's speed follows its power, 1/s (0.2 s time constant). PLACEHOLDER. */
    static final double DRIVE_RESPONSE_PER_S = 5.0;
    static final double TRACK_WIDTH_IN = 12.96;                 // MEASURED FROM CAD (wheel centres y +/-6.48)
    static final double WHEEL_BASE_IN = 10.39;                  // MEASURED FROM CAD (wheel centres 10.39 in apart)

    // ----- Shooters: goBILDA 5000-series 6000 rpm, 1:1 assumed -----
    static final double SHOOTER_MAX_TICKS_PER_S = 2800.0;       // SPEC (6000 rpm x 28 / 60)
    static final double SHOOTER_TICKS_PER_REV = 28.0;           // SPEC
    /** How fast a flywheel follows its power, 1/s. A loaded flywheel is slower than a bare shaft. PLACEHOLDER. */
    static final double SHOOTER_RESPONSE_PER_S = 3.0;

    // ----- Intake -----
    static final double INTAKE_MAX_TICKS_PER_S = 2800.0;        // PLACEHOLDER
    static final double INTAKE_RESPONSE_PER_S = 8.0;            // PLACEHOLDER

    // ----- Pinpoint: where the pods really sit. Must match what pedro/Constants tells it. -----
    static final double X_POD_OFFSET_IN = -2.84;                // MEASURED FROM CAD (same as Constants)
    static final double Y_POD_OFFSET_IN = -5.1;                 // APPROX, from a CAD screenshot (same as Constants)
    static final double POD_TICKS_PER_MM = 19.89436789;         // SPEC (goBILDA 4-bar pod)

    /** The chassis. */
    public static final DrivetrainSimConfig DRIVETRAIN = new DrivetrainSimConfig() {
        @Override public double trackWidth() { return TRACK_WIDTH_IN; }
        @Override public double wheelBase() { return WHEEL_BASE_IN; }
        @Override public double wheelRadius() { return WHEEL_RADIUS_IN; }
        @Override public double ticksPerInch() { return DRIVE_TICKS_PER_INCH; }
        @Override public double maxVelocityTicksPerSecond() { return DRIVE_MAX_TICKS_PER_S; }
        @Override public double maxAccel() { return DRIVE_RESPONSE_PER_S; }
        /**
         * Which way each motor is BOLTED IN, [FL, FR, BL, BR]. The code reverses the right pair in
         * software (BioHorsRobot, pedro/Constants), so on the metal the right pair is mirrored.
         */
        @Override public double[] wheelMountingSigns() { return new double[] {1.0, -1.0, 1.0, -1.0}; }
        // MEASURED FROM CAD: the perimeter's inside faces (autos/field sizeIn, FIRST's field CAD)
        @Override public double fieldHalfWidth() { return 70.674; }
        @Override public double fieldHalfHeight() { return 70.674; }
    };

    /** Gate servo: positions are servo command units, so the command map is the identity. */
    static final PositionalServoSimConfig GATE = new PositionalServoSimConfig() {
        @Override public double timeConstant() { return 0.05; }    // PLACEHOLDER
        /** Full sweep in about 0.5 s. PLACEHOLDER: gate transit is NEEDS MEASUREMENT. */
        @Override public double maxSpeed() { return 2.0; }
        @Override public double minPosition() { return 0.0; }
        @Override public double maxPosition() { return 1.0; }
        @Override public double positionAtCommandZero() { return 0.0; }
        @Override public double positionAtCommandOne() { return 1.0; }
        @Override public double commandDeadband() { return 0.005; } // PLACEHOLDER
    };

    public static final String[] DRIVE_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};

    public static final SimRobotConfig ROBOT_CONFIG = new SimRobotConfig() {
        @Override public String[] driveMotorNames() { return DRIVE_NAMES; }
        @Override public String odometryName() { return "pinpoint"; }
        @Override public DrivetrainSimConfig drivetrain() { return DRIVETRAIN; }
    };

    public final FakeHardwareMap hardwareMap = new FakeHardwareMap();
    public final FakeMotor[] driveMotors = new FakeMotor[4];
    public final FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
    public final FakeMotor intakeLeft = new FakeMotor(INTAKE_MAX_TICKS_PER_S, INTAKE_RESPONSE_PER_S);
    public final FakeMotor intakeRight = new FakeMotor(INTAKE_MAX_TICKS_PER_S, INTAKE_RESPONSE_PER_S);
    public final FakeMotor pollenShooter =
            new FakeMotor(SHOOTER_MAX_TICKS_PER_S, SHOOTER_RESPONSE_PER_S, SHOOTER_TICKS_PER_REV);
    public final FakeMotor nectarShooter =
            new FakeMotor(SHOOTER_MAX_TICKS_PER_S, SHOOTER_RESPONSE_PER_S, SHOOTER_TICKS_PER_REV);
    public final FakeServo pollenGate = new FakeServo();
    public final FakeServo nectarGate = new FakeServo();
    public final FakeServo flowerLeft = new FakeServo();
    public final FakeServo flowerRight = new FakeServo();
    public final FakeVoltageSensor battery = new FakeVoltageSensor(12.0);

    /** Chassis physics. Its pose is the robot's true field pose. */
    public final MecanumDrivePlant drivePlant;
    /** What the gates are really doing (the robot cannot see this). */
    public final PositionalServoPlant pollenGatePlant;
    public final PositionalServoPlant nectarGatePlant;
    /** The flower bar's two servos (the same transit model as a gate: PLACEHOLDER). */
    public final PositionalServoPlant flowerLeftPlant;
    public final PositionalServoPlant flowerRightPlant;

    public BioHorsSim() {
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = new FakeMotor(DRIVE_MAX_TICKS_PER_S, DRIVE_RESPONSE_PER_S, DRIVE_TICKS_PER_REV);
            hardwareMap.register(DRIVE_NAMES[i], driveMotors[i]);
        }
        pinpoint.setPhysicalPodMount(X_POD_OFFSET_IN * MM_PER_IN, Y_POD_OFFSET_IN * MM_PER_IN,
                GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD,
                POD_TICKS_PER_MM);
        hardwareMap.register("pinpoint", pinpoint);
        hardwareMap.register("intakeLeft", intakeLeft);
        hardwareMap.register("intakeRight", intakeRight);
        hardwareMap.register("shooterRight", pollenShooter); // pollen = shooterRight (MechanismConstants)
        hardwareMap.register("shooterLeft", nectarShooter);
        hardwareMap.register("gateRight", pollenGate);       // pollen gate
        hardwareMap.register("gateLeft", nectarGate);
        hardwareMap.register("flowerLeft", flowerLeft);     // flower bar (MechanismConstants)
        hardwareMap.register("flowerRight", flowerRight);
        hardwareMap.register("battery", battery);

        drivePlant = new MecanumDrivePlant(driveMotors, DRIVETRAIN);
        pollenGatePlant = new PositionalServoPlant(pollenGate, GATE);
        nectarGatePlant = new PositionalServoPlant(nectarGate, GATE);
        flowerLeftPlant = new PositionalServoPlant(flowerLeft, GATE);
        flowerRightPlant = new PositionalServoPlant(flowerRight, GATE);
    }

    /** Puts the chassis at a field pose without driving there; the Pinpoint does not count it as motion. */
    public void placeAt(double xIn, double yIn, double headingRad) {
        drivePlant.setPose(xIn, yIn, headingRad);
        pinpoint.attachChassisAt(xIn * MM_PER_IN, yIn * MM_PER_IN, headingRad);
    }

    /** The world's response to one tick of robot code. Pass to OpModeRunner.eachTick. */
    public void step(double dt) {
        drivePlant.update(dt);
        pinpoint.moveChassisTo(drivePlant.getX() * MM_PER_IN, drivePlant.getY() * MM_PER_IN,
                drivePlant.getHeading());
        pollenGatePlant.update(dt);
        nectarGatePlant.update(dt);
        flowerLeftPlant.update(dt);
        flowerRightPlant.update(dt);
        simTimeS += dt;
        if (batterySag) applyBatterySag();
    }

    // ----- Battery sag (off unless a test turns it on) -----
    /** Resting battery voltage when sag is on. PLACEHOLDER. */
    static final double BATTERY_REST_V = 12.6;
    /** Volts lost per unit of shooter power drawn (both shooters add). PLACEHOLDER: ~0.9 V with both at full. */
    static final double BATTERY_SAG_V_PER_POWER = 0.45;
    private boolean batterySag = false;
    private double restStartV = BATTERY_REST_V;
    private double drainVPerS = 0.0;
    private double simTimeS = 0.0;

    /**
     * Turns on a battery that sags with the shooters' combined power, and flywheels that slow in
     * proportion to the voltage, as a real DC motor does (speed ~ power x V / 12). Used to show
     * that voltage-compensated feedforward holds both shooters on target at once.
     */
    public void enableShooterBatterySag() {
        batterySag = true;
    }

    /**
     * As {@link #enableShooterBatterySag()}, with a battery that also drains over the run: its
     * resting voltage starts at {@code startV} and falls by {@code drainVPerS} each second, as a
     * match battery does (about 12.8 V fresh to about 11.5 V tired). PLACEHOLDER numbers.
     */
    public void enableShooterBatterySag(double startV, double drainVPerS) {
        batterySag = true;
        restStartV = startV;
        this.drainVPerS = drainVPerS;
    }

    /** Applies the sag for this tick. Call from step(), before the devices next update. */
    private void applyBatterySag() {
        double pP = Math.abs(pollenShooter.getPower());
        double pN = Math.abs(nectarShooter.getPower());
        double rest = restStartV - drainVPerS * simTimeS;
        double v = rest - BATTERY_SAG_V_PER_POWER * (pP + pN);
        battery.setVoltage(v);
        // The fake motor's steady speed is (power + external torque fraction) x free speed, capped
        // at free speed (measured: a positive fraction ASSISTS). A real motor at voltage v runs at
        // power x v / 12, so the fraction is power x (v / 12 - 1): negative, a load, below 12 V.
        // Limit: the fake cannot exceed its 12 V free speed, so above 12 V full power reads low.
        pollenShooter.setExternalTorqueFraction(pollenShooter.getPower() * (v / 12.0 - 1.0));
        nectarShooter.setExternalTorqueFraction(nectarShooter.getPower() * (v / 12.0 - 1.0));
    }

    /** Pollen shooter's true flywheel speed, rpm. */
    public double pollenShooterRpm() {
        return pollenShooter.getPhysicalVelocityTicksPerSecond() * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    public double nectarShooterRpm() {
        return nectarShooter.getPhysicalVelocityTicksPerSecond() * 60.0 / SHOOTER_TICKS_PER_REV;
    }
}
