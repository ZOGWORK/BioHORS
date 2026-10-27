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
}
