package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Hardware names and tuning values for the shooters and gates, in one place.
 *
 * Every number carries a provenance label:
 *   SPEC              from the manufacturer's spec sheet
 *   PLACEHOLDER       made up so the code runs; NOT a real value
 *   NEEDS MEASUREMENT must be measured on the robot before it means anything
 *
 * Nothing here is measured yet. Do not treat any value as tuned.
 */
public final class MechanismConstants {

    private MechanismConstants() {
    }

    // ===== Hardware map names (must match the Robot Controller config) =====
    // CONFIRMED by the team (Oct 27): one motor per shooter. Top-down with the intake at the
    // front (up), the NECTAR shooter is on the LEFT and the POLLEN shooter on the RIGHT.
    public static final String POLLEN_SHOOTER_NAME = "shooterRight";
    public static final String NECTAR_SHOOTER_NAME = "shooterLeft";
    public static final String POLLEN_GATE_NAME = "gateRight"; // confirmed Oct 27: right gate feeds pollen
    public static final String NECTAR_GATE_NAME = "gateLeft";  // confirmed Oct 27: left gate feeds nectar
    // The flower bar's two servos (mounted facing away from each other; the right one is reversed
    // in FlowerBarController). NEEDS CONFIRMATION: add them to the Robot Controller config by these names.
    public static final String FLOWER_BAR_LEFT_NAME = "flowerLeft";
    public static final String FLOWER_BAR_RIGHT_NAME = "flowerRight";

    // ===== Shooter directions =====
    // Both shooters physically spin the SAME way (team spec). The motor
    // Direction values that produce that depend on how each motor is mounted.
    // NEEDS MEASUREMENT: spin each shooter on the robot and flip as needed.
    public static DcMotorSimple.Direction POLLEN_SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction NECTAR_SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // ===== goBILDA 5000-series 6000 RPM motor (1:1) =====
    // SPEC: 28 encoder counts per motor-shaft revolution; 6000 RPM free speed
    // at 12 V => 6000 * 28 / 60 = 2800 motor-encoder ticks/sec.
    // ticksPerRev is per FLYWHEEL revolution, so 28 holds only if the flywheel
    // is direct-driven (1:1); with a belt/gear ratio, multiply ticksPerRev by
    // it. The 2800 ticks/sec feedforward max is the motor's own free speed and
    // does NOT change with an external ratio. CONFIRMED by the team (Oct 27): the flywheels are 1:1.
    private static final double GOBILDA_6000_TICKS_PER_REV = 28.0;           // SPEC
    private static final double GOBILDA_6000_FREE_SPEED_TICKS_PER_SEC = 2800.0; // SPEC (no load, 12 V)

    /** Pollen shooter. Used in auto and teleop. */
    public static final ShooterController.Config POLLEN_SHOOTER = new ShooterController.Config();

    /**
     * The pollen flywheel's speed in AUTO, from the scoring spots 48 in from the hive axle. PLACEHOLDER,
     * SET FROM SIM: the middle of the 1650-1800 rpm that seat a pollen from there in the sim's physics,
     * at the hood's 75 deg exit (MEASURED FROM CAD) and a WHAT-IF ball speed of 0.8 of the rim speed (at
     * the physics placeholder 0.5 it would be about 2700). Film a shot, measure its speed, then re-run
     * ShotSweepTest and set this from it.
     */
    public static double AUTO_POLLEN_RPM = 1750.0;

    /** Nectar shooter. Teleop / endgame only. Runs at its own (different) speed. */
    public static final ShooterController.Config NECTAR_SHOOTER = new ShooterController.Config();

    static {
        POLLEN_SHOOTER.label = "Pollen";
        applySharedPlaceholders(POLLEN_SHOOTER);
        // SET BY HAND: the team's test target (Oct 27). The RPM that scores is a field test (the
        // Shooter Tuner only makes the shooter hold whatever this is).
        POLLEN_SHOOTER.targetRpm = 1600.0;
        // ---- Pollen tuned values: paste the "Pollen" lines the Shooter Tuner OpMode prints here ----

        NECTAR_SHOOTER.label = "Nectar";
        applySharedPlaceholders(NECTAR_SHOOTER);
        // SET BY HAND: the team's test target (Oct 27), the same start as pollen. Nectar is
        // expected to settle at its own speed after a field test; trim it with X / A.
        NECTAR_SHOOTER.targetRpm = 1600.0;
        // ---- Nectar tuned values: paste the "Nectar" lines the Shooter Tuner OpMode prints here ----
    }

    /**
     * Starting values both shooters share until each is tuned on its own.
     * Same motor model, so the spec values match; every gain is a placeholder.
     */
    private static void applySharedPlaceholders(ShooterController.Config c) {
        c.ticksPerRev = GOBILDA_6000_TICKS_PER_REV;                       // SPEC; 1:1 confirmed
        c.ffReferenceVoltage = 12.0;                                       // SPEC (rated voltage)
        c.ffReferenceMaxTicksPerSec = GOBILDA_6000_FREE_SPEED_TICKS_PER_SEC; // SPEC; NEEDS MEASUREMENT under load
        c.kP = 0.001;                  // PLACEHOLDER
        c.kI = 0.0005;                 // PLACEHOLDER
        c.kD = 0.0;                    // PLACEHOLDER
        c.kF = 1.0;                    // PLACEHOLDER
        c.kS = 0.0;                    // PLACEHOLDER: static friction off until tuned
        c.integralLimit = 50.0;        // PLACEHOLDER
        c.derivativeAlpha = 0.9;       // PLACEHOLDER
        c.rpmFilterAlpha = 0.72;       // PLACEHOLDER
        c.powerSmoothingAlpha = 0.5;   // PLACEHOLDER
        c.rpmTolerance = 75.0;         // PLACEHOLDER
    }

    // ===== Teleop flywheel speed by distance to the hive cell (aim/DistanceSpeedTable) =====
    // FILL IN WITH FIELD MEASUREMENTS: for each shooter, the rpm that scores from each distance
    // (inches, from the drive centre to the cell's aim point), distances increasing. The teleop
    // interpolates between points and holds the end points beyond them; the trim adds on top.
    // Until measured, each table is ONE flat point at that shooter's fixed target rpm, so teleop
    // runs exactly as before. A one-point table is a constant; its distance does not matter.
    public static double[] POLLEN_SPEED_TABLE_DISTANCE_IN = {48.0};                  // PLACEHOLDER: one flat point
    public static double[] POLLEN_SPEED_TABLE_RPM = {POLLEN_SHOOTER.targetRpm};      // = the fixed target (SET BY HAND)
    public static double[] NECTAR_SPEED_TABLE_DISTANCE_IN = {48.0};                  // PLACEHOLDER: one flat point
    public static double[] NECTAR_SPEED_TABLE_RPM = {NECTAR_SHOOTER.targetRpm};      // = the fixed target (SET BY HAND)
    /**
     * The teleop moves a shooter's target only when the table's rpm has moved at least this far
     * (a trim press always applies). ShooterController clears its integral on every target change,
     * so without this a multi-point table would reset it every loop as the robot drifts. SET BY HAND.
     */
    public static double SPEED_TABLE_UPDATE_STEP_RPM = 10.0;

    // ===== Teleop trim: each shooter its own step (rpm per press) =====
    public static double POLLEN_TRIM_STEP_RPM = 50.0; // SET BY HAND
    public static double NECTAR_TRIM_STEP_RPM = 50.0; // SET BY HAND

    // ===== Gate servo positions (per gate) =====
    // PLACEHOLDER: both gates start at the old shared values. Move each gate
    // by hand from the driver station and record its own open/closed positions.
    public static double POLLEN_GATE_OPEN = 0.67;    // PLACEHOLDER
    public static double POLLEN_GATE_CLOSED = 0.485; // PLACEHOLDER
    public static double NECTAR_GATE_OPEN = 0.67;    // PLACEHOLDER
    public static double NECTAR_GATE_CLOSED = 0.485; // PLACEHOLDER

    // ===== Flower bar servo positions (both servos get the same number) =====
    // PLACEHOLDER: arbitrary until set on the robot. STOWED folds the bar up (the robot at its
    // shortest, as it starts the match); DOWN puts it out and low for a flower's opening;
    // SHOOTING is DOWN raised slightly, so shots clear it.
    public static double FLOWER_BAR_STOWED = 0.05;   // PLACEHOLDER: folded well back, so the bar and its side plates sit
                                                     // inside the frame and clear of the wall when the robot runs close to it
    public static double FLOWER_BAR_DOWN = 0.80;     // PLACEHOLDER
    public static double FLOWER_BAR_SHOOTING = 0.65; // PLACEHOLDER
    // ===== Pollen count from the intake's speed (IntakeLoadSensor) =====
    // Both intake motors have encoders. Pollen aboard press on the rollers, so the intake slows.
    public static final int HOPPER_CAPACITY = 4;                // SET BY HAND: confirmed by the team (G407)
    public static boolean INTAKE_SENSOR_ENABLED = true;          // false: fall back to the time estimate
    /** Average intake speed running inward, empty, ticks/s. PLACEHOLDER (the sim's): read it on the robot. */
    public static double INTAKE_FREE_TICKS_PER_S = 2800.0;
    /** Intake speed at or below which the robot is loaded (full). CARRIED OVER from MAINHORS26's
     *  IntakeBallDetector (DECODE): the same team's field-tested intake threshold. */
    public static double INTAKE_LOADED_TICKS_PER_S = 2100.0;
    /** Fraction of the free speed lost per pollen aboard: set so a full hopper reads exactly the
     *  loaded threshold, (free - loaded) / free / capacity = 0.0625. SET FROM the two above. */
    public static double INTAKE_SLOWDOWN_PER_PIECE =
            (INTAKE_FREE_TICKS_PER_S - INTAKE_LOADED_TICKS_PER_S) / INTAKE_FREE_TICKS_PER_S / HOPPER_CAPACITY;
    public static double INTAKE_SENSE_MIN_POWER = 0.5;          // SET BY HAND: read only while pulling in hard
    public static double INTAKE_SENSE_SETTLE_S = 0.4;           // PLACEHOLDER: ignore the spin-up after starting
    public static double INTAKE_SENSE_DEBOUNCE_S = 0.15;        // CARRIED OVER: MAINHORS26's SPEED_STABLE_HOLD_SECONDS
    public static double INTAKE_SENSE_FILTER_ALPHA = 0.3;       // PLACEHOLDER: speed smoothing per loop
    // ===== Shooter encoder check (ShooterController) =====
    /** Only judged while the motor is given at least this power. SET BY HAND: well above the
     *  ~0.27 a 1600 rpm target needs at full speed, so only a hard push counts. */
    public static double SHOOTER_ENCODER_CHECK_POWER = 0.3;
    /** Below this reading, rpm, the encoder counts as silent. SET BY HAND: a turning flywheel reads
     *  hundreds of rpm within a fraction of a second. */
    public static double SHOOTER_ENCODER_DEAD_RPM = 50.0;
    /** Silent for this long while pushed, seconds, means dead. PLACEHOLDER: NEEDS MEASUREMENT (time a
     *  cold spin-up to 50 rpm on the robot; this must be well above it). */
    public static double SHOOTER_ENCODER_DEAD_S = 0.5;

    // ===== Auto spin-up check =====
    /** At the spin-up timeout, the auto fires only if the pollen flywheel has reached at least this
     *  fraction of its target; below it (a dead motor or encoder) the volley is skipped and the pollen
     *  stay aboard. PLACEHOLDER: NEEDS MEASUREMENT (film a shot at 80 % speed: does it still score?). */
    public static double SPINUP_MIN_FRACTION = 0.8;

    // ===== Teleop intake jam reverse (IntakeJamGuard) =====
    // A jam is the intake held inward but nearly stopped: far slower than a FULL robot
    // (INTAKE_LOADED_TICKS_PER_S), so a full hopper is never read as a jam.
    /** Below this average intake speed, ticks/s, the intake counts as stalled. PLACEHOLDER: a quarter of
     *  the free speed. NEEDS MEASUREMENT: hold a pollen against the rollers and read "Intake speed". */
    public static double INTAKE_JAM_TICKS_PER_S = 700.0;
    /** The stall must last this long before the intake reverses, seconds. PLACEHOLDER. */
    public static double INTAKE_JAM_DETECT_S = 0.25;
    /** How long a jam reverse runs before pulling in again, seconds. PLACEHOLDER. */
    public static double INTAKE_JAM_REVERSE_S = 0.3;

    /** In auto, stop waiting for more pollen once none has come in for this long. PLACEHOLDER. */
    public static double COLLECT_NO_PROGRESS_S = 0.5;

    /** Time for the bar to reach SHOOTING before a volley feeds, seconds. PLACEHOLDER: film it. */
    public static double FLOWER_BAR_SETTLE_S = 0.25;

    /** Time for the bar to swing from STOWED to DOWN before the robot drives into a flower, seconds.
     *  PLACEHOLDER: the sim's servo takes 0.38 s; film the real one. The Zenith auto's waitBarDown
     *  step must say the same (BarDropWaitTest). */
    public static double FLOWER_BAR_DROP_S = 0.5;
}
