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
    // Which shooter motor is which is NEEDS MEASUREMENT: confirm on the robot
    // that shooterRight is the pollen shooter (same side as its gate).
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
    // at 12 V => 6000 * 28 / 60 = 2800 ticks/sec. These hold for the FLYWHEEL
    // only if it is direct-driven (1:1). If there is a belt/gear ratio,
    // multiply ticksPerRev by it. NEEDS MEASUREMENT: confirm the ratio.
    private static final double GOBILDA_6000_TICKS_PER_REV = 28.0;           // SPEC
    private static final double GOBILDA_6000_FREE_SPEED_TICKS_PER_SEC = 2800.0; // SPEC (no load, 12 V)

    /** Pollen shooter. Used in auto and teleop. */
    public static final ShooterController.Config POLLEN_SHOOTER = new ShooterController.Config();

    /** Nectar shooter. Teleop / endgame only. Runs at its own (different) speed. */
    public static final ShooterController.Config NECTAR_SHOOTER = new ShooterController.Config();

    static {
        POLLEN_SHOOTER.label = "Pollen";
        applySharedPlaceholders(POLLEN_SHOOTER);
        POLLEN_SHOOTER.targetRpm = 2000.0; // PLACEHOLDER

        NECTAR_SHOOTER.label = "Nectar";
        applySharedPlaceholders(NECTAR_SHOOTER);
        // PLACEHOLDER. Nectar runs at a different speed from pollen; the real
        // number is unknown, so it starts at the same placeholder.
        NECTAR_SHOOTER.targetRpm = 2000.0;
    }

    /**
     * Starting values both shooters share until each is tuned on its own.
     * Same motor model, so the spec values match; every gain is a placeholder.
     */
    private static void applySharedPlaceholders(ShooterController.Config c) {
        c.ticksPerRev = GOBILDA_6000_TICKS_PER_REV;                       // SPEC (assumes 1:1)
        c.ffReferenceVoltage = 12.0;                                       // SPEC (rated voltage)
        c.ffReferenceMaxTicksPerSec = GOBILDA_6000_FREE_SPEED_TICKS_PER_SEC; // SPEC; NEEDS MEASUREMENT under load
        c.kP = 0.001;                  // PLACEHOLDER
        c.kI = 0.0005;                 // PLACEHOLDER
        c.kD = 0.0;                    // PLACEHOLDER
        c.kF = 1.0;                    // PLACEHOLDER
        c.integralLimit = 50.0;        // PLACEHOLDER
        c.derivativeAlpha = 0.9;       // PLACEHOLDER
        c.rpmFilterAlpha = 0.72;       // PLACEHOLDER
        c.powerSmoothingAlpha = 0.5;   // PLACEHOLDER
        c.rpmTolerance = 75.0;         // PLACEHOLDER
    }

    // ===== Gate servo positions (per gate) =====
    // PLACEHOLDER: both gates start at the old shared values. Move each gate
    // by hand from the driver station and record its own open/closed positions.
    public static double POLLEN_GATE_OPEN = 0.67;    // PLACEHOLDER
    public static double POLLEN_GATE_CLOSED = 0.485; // PLACEHOLDER
    public static double NECTAR_GATE_OPEN = 0.67;    // PLACEHOLDER
    public static double NECTAR_GATE_CLOSED = 0.485; // PLACEHOLDER
}
