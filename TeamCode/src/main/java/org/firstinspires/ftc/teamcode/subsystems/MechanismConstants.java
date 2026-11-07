package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

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
@Configurable
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
     * The pollen flywheel's speed in AUTO, from the scoring spots 48 in from the hive axle. SET BY HAND
     * (the team, 2026-11-05): the teleop's 3100 (was 1750, SET FROM SIM at a WHAT-IF ball speed of 0.8
     * of the rim speed; at the physics placeholder 0.5 it would be about 2700). Check it scores from
     * the auto's spots; if not, film a shot, re-run ShotSweepTest and set this from it.
     */
    public static double AUTO_POLLEN_RPM = 3100.0;

    /** Nectar shooter. Teleop / endgame only. Runs at its own (different) speed. */
    public static final ShooterController.Config NECTAR_SHOOTER = new ShooterController.Config();

    static {
        POLLEN_SHOOTER.label = "Pollen";
        applySharedPlaceholders(POLLEN_SHOOTER);
        // SET BY HAND (the team, 2026-11-06; 3100 from 2026-11-05, 1600 from Oct 27). The RPM that scores is a field
        // test (the Shooter Tuner only makes the shooter hold whatever this is).
        POLLEN_SHOOTER.targetRpm = 3050.0;
        // ---- Pollen tuned values ----
        // Pollen: CALIBRATED FROM ROBOT by FtcAutoTune v1.4.3 (2026-11-05), PD (classic ZN) candidate,
        // Ku 0.019099 Tu 0.2211 s, kF sweep at 13.52 V; kS not measured.
        POLLEN_SHOOTER.ffReferenceVoltage = 12.0;
        POLLEN_SHOOTER.ffReferenceMaxTicksPerSec = 2250.8;
        POLLEN_SHOOTER.kF = 1.0;
        POLLEN_SHOOTER.kS = 0.0;
        // P and D: SET BY HAND (the team, 2026-11-06: higher, for rapid fire; kF kept): 1.5x the autotune's,
        // kP 0.56 Ku (Ku measured on the robot above), well short of where the loop oscillates. NEEDS
        // CHECKING on the robot: fire a volley and watch the rpm recover without ringing.
        POLLEN_SHOOTER.kP = 0.010700;   // was 0.007130
        POLLEN_SHOOTER.kI = 0.000000;
        POLLEN_SHOOTER.kD = 0.000296;   // was 0.000197

        NECTAR_SHOOTER.label = "Nectar";
        applySharedPlaceholders(NECTAR_SHOOTER);
        // SET BY HAND (the team, 2026-11-06; 3100 from 2026-11-05, 1600 before): its own speed now,
        // higher than pollen's.
        NECTAR_SHOOTER.targetRpm = 3200.0;
        // ---- Nectar tuned values ----
        // Nectar: CALIBRATED FROM ROBOT by FtcAutoTune v1.4.3 (2026-11-05), PD (classic ZN) candidate,
        // Ku 0.019099 Tu 0.2913 s, kF sweep at 13.41 V; kS not measured. (Replaces the kF 1.2 set by
        // hand: the measured max speed already makes up the shortfall.)
        NECTAR_SHOOTER.ffReferenceVoltage = 12.0;
        NECTAR_SHOOTER.ffReferenceMaxTicksPerSec = 2229.4;
        NECTAR_SHOOTER.kF = 1.0;
        NECTAR_SHOOTER.kS = 0.0;
        // P and D: SET BY HAND (the team, 2026-11-06: higher, for rapid fire; kF kept): 1.5x the autotune's,
        // kP 0.56 Ku (Ku measured on the robot above), well short of where the loop oscillates. NEEDS
        // CHECKING on the robot: fire a volley and watch the rpm recover without ringing.
        NECTAR_SHOOTER.kP = 0.010700;   // was 0.007130
        NECTAR_SHOOTER.kI = 0.000000;
        NECTAR_SHOOTER.kD = 0.000390;   // was 0.000260
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

    // ===== Teleop trim: dpad left/right moves BOTH shooters by this much (rpm per press) =====
    public static double SHOOTER_TRIM_STEP_RPM = 50.0; // SET BY HAND

    // ===== Gate servo positions (per gate) =====
    // SET BY HAND (the team, on the robot in teleop, 2026-11-05). Teleop and both autos read these.
    public static double POLLEN_GATE_OPEN = 0.485;   // SET BY HAND
    public static double POLLEN_GATE_CLOSED = 0.35;   // SET BY HAND
    public static double NECTAR_GATE_OPEN = 0.485;   // SET BY HAND
    public static double NECTAR_GATE_CLOSED = 0.2;   // SET BY HAND

    // ===== Flower bar servo positions =====
    // The number is the LEFT servo's. The right servo faces the other way and is REVERSE, so it
    // gets 1 - this (left 0.35 = right 0.65): the two always add up to 1. START is where both sit
    // at init, until the first command; STOWED is the bar UP; DOWN is out and low for a flower's
    // opening; SHOOTING (autos only) is DOWN raised a little, so shots clear it.
    // Up and down swapped on 2026-11-05: on the robot the low number is down and 0.80 up (the
    // team, driving it); SHOOTING keeps its 0.15 lift off DOWN.
    public static double FLOWER_BAR_START = 0.80;    // = STOWED (the team, 2026-11-05)
    public static double FLOWER_BAR_STOWED = 0.80;   // SET BY HAND (the team, on the robot): folded back, so the bar and its
                                                     // side plates sit inside the frame and clear of the wall
    public static double FLOWER_BAR_DOWN = 0.0;      // SET BY HAND (the team, on the robot, 2026-11-05)
    public static double FLOWER_BAR_SHOOTING = 0.15; // PLACEHOLDER (autos only): DOWN raised 0.15
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
    /** Intake power for the whole AUTO except feeding a volley, in [0, 1]: it runs from the start to
     *  the park without stopping (between volleys it keeps pulling in; the gate holds the pollen).
     *  SET BY HAND (the team, 2026-11-06: 0.67; was 0.6). Teleop still uses IntakeController.INTAKE_POWER. */
    public static double AUTO_INTAKE_POWER = 0.67;
    /** Intake power while a volley feeds pollen through the open gate in AUTO, in [0, 1]. SET BY HAND
     *  (the team, 2026-11-06: at 0.6 the intake did not turn while shooting on the robot; full power,
     *  as teleop feeds). */
    public static double AUTO_FEED_POWER = 1.0;
    public static double INTAKE_SENSE_MIN_POWER = 0.5;          // SET BY HAND: read only while pulling in hard
    public static double INTAKE_SENSE_SETTLE_S = 0.4;           // PLACEHOLDER: ignore the spin-up after starting
    public static double INTAKE_SENSE_DEBOUNCE_S = 0.15;        // CARRIED OVER: MAINHORS26's SPEED_STABLE_HOLD_SECONDS
    public static double INTAKE_SENSE_FILTER_ALPHA = 0.3;       // PLACEHOLDER: speed smoothing per loop
    // ===== Teleop idle flywheel speed =====
    /** Between volleys the teleop keeps both flywheels turning slowly, so aiming spins them up
     *  sooner. false: they stop, as before. Idle never counts as ON for the fire interlock.
     *  OFF for now (the team, 2026-10-31): turn it back on once the idle rpm is measured. */
    public static boolean SHOOTER_IDLE_ENABLED = false;
    /** The idle speed, flywheel rpm. PLACEHOLDER: about half the firing speed; NEEDS MEASUREMENT
     *  (time the spin-up from idle; lower it if the battery suffers). */
    public static double SHOOTER_IDLE_RPM = 800.0;
    /** Teleop display: a switched-off shooter still turning faster than this, rpm, shows "OFF, coasting"
     *  (the motors FLOAT, so a flywheel takes a while to stop). SET BY HAND: display only. */
    public static double SHOOTER_COAST_SHOW_RPM = 100.0;

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
    /** The stall must last this long before the intake reverses, seconds. SET BY HAND (the team,
     *  2026-11-06: no reversing on a short stall, only one that lasts more than 5 s; was 0.25). */
    public static double INTAKE_JAM_DETECT_S = 5.0;
    /** How long a jam reverse runs before pulling in again, seconds. PLACEHOLDER. */
    public static double INTAKE_JAM_REVERSE_S = 0.3;
    /** Reverses in a row that do not clear a jam before the guard stops the intake. SET BY HAND. */
    public static int INTAKE_JAM_GIVE_UP_REVERSES = 3;

    /** In auto, stop waiting for more pollen once none has come in for this long. PLACEHOLDER. */
    public static double COLLECT_NO_PROGRESS_S = 0.5;

    /** Time for the bar to reach SHOOTING before a volley feeds, seconds. PLACEHOLDER: film it. */
    public static double FLOWER_BAR_SETTLE_S = 0.25;

    /** Time for the bar to swing from STOWED to DOWN before the robot drives into a flower, seconds.
     *  PLACEHOLDER: the sim's servo takes 0.38 s; film the real one. The Zenith auto's waitBarDown
     *  step must say the same (BarDropWaitTest). */
    public static double FLOWER_BAR_DROP_S = 0.5;
}
