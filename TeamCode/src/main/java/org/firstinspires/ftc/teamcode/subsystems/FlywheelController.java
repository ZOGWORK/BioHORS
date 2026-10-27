package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Two-motor flywheel (shooter) velocity controller using the motors' built-in
 * encoders. Both motors get the same PIDF output power; opposite spin comes
 * from the motor directions set in the OpMode (left REVERSE, right FORWARD),
 * not from different powers.
 *
 * ============================================================================
 * TODO: TUNE THIS. The PIDF values below are PLACEHOLDERS. They are
 * intentionally NOT the HORS values -- do not copy HORS's numbers here.
 * Retune kP, kI, kD, kF, TARGET_RPM, and TICKS_PER_REV on THIS robot
 * before a match, or the flywheel will not hold speed.
 * ============================================================================
 */
public class FlywheelController {

    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final Telemetry telemetry; // nullable
    private final VoltageSensor voltageSensor; // nullable
    private final ElapsedTime timer = new ElapsedTime();

    // TODO: CHANGE THIS -- encoder ticks per OUTPUT revolution of YOUR flywheel
    // motors (motor spec ticks x gear ratio). Verify against the motor spec
    // page; do not assume another robot's number.
    public static double TICKS_PER_REV = 28.0;

    // TODO: TUNE THESE -- placeholder PIDF, NOT the HORS values.
    public static double kP = 0.001;
    public static double kI = 0.0005;
    public static double kD = 0.0;
    public static double kF = 1.0;
    public static double integralLimit = 50.0;
    public static double derivativeAlpha = 0.9;
    public static double rpmFilterAlpha = 0.72;
    public static double powerSmoothingAlpha = 0.5;

    // TODO: TUNE THIS -- placeholder target speed, NOT the HORS target.
    public static double TARGET_RPM = 2000.0;
    public static double RPM_TOLERANCE = 75.0;

    // Feedforward reference: max ticks/sec at the reference voltage.
    // TODO: MEASURE THIS -- spin the flywheel wide open, read getVelocity()
    // at full battery, and put that number here.
    public static double FF_REFERENCE_VOLTAGE = 12.0;
    public static double FF_REFERENCE_MAX_TICKS_PER_SEC = 5000.0;

    // --- Internal state ---
    private double targetRpm = TARGET_RPM;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastDerivativeEstimate = 0.0;
    private int lastPos = 0;
    private double currentRpm = 0.0;
    private double lastAppliedPower = 0.0;

    private boolean shooterOn = false;
    private boolean lastAtTarget = false;
    private boolean justReachedTargetFlag = false;

    public FlywheelController(DcMotor shooterLeft, DcMotor shooterRight,
                              Telemetry telemetry, VoltageSensor voltageSensor) {
        if (!(shooterLeft instanceof DcMotorEx)) {
            throw new IllegalArgumentException("Left shooter must be a DcMotorEx");
        }
        if (!(shooterRight instanceof DcMotorEx)) {
            throw new IllegalArgumentException("Right shooter must be a DcMotorEx");
        }
        this.shooterLeft = (DcMotorEx) shooterLeft;
        this.shooterRight = (DcMotorEx) shooterRight;
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;

        try {
            this.shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("Flywheel.init", "config failed: " + e.getMessage());
        }

        timer.reset();
        lastPos = this.shooterLeft.getCurrentPosition();
    }

    public FlywheelController(DcMotor shooterLeft, DcMotor shooterRight, Telemetry telemetry) {
        this(shooterLeft, shooterRight, telemetry, null);
    }

    public void setTargetRpm(double rpm) {
        if (rpm != targetRpm) {
            // Reset on setpoint changes so the integral does not carry over.
            integralSum = 0.0;
            lastError = 0.0;
            lastDerivativeEstimate = 0.0;
        }
        targetRpm = rpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void adjustTargetRpm(double delta) {
        setTargetRpm(Math.max(0.0, targetRpm + delta));
    }

    public boolean isAtSpeed() {
        return Math.abs(targetRpm - currentRpm) <= RPM_TOLERANCE;
    }

    public boolean isAtTarget() {
        return isAtSpeed();
    }

    public void toggleShooterOn() {
        shooterOn = !shooterOn;
    }

    public void setShooterOn(boolean on) {
        shooterOn = on;
    }

    public boolean isShooterOn() {
        return shooterOn;
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public double getLastAppliedPower() {
        return lastAppliedPower;
    }

    public boolean justReachedTarget() {
        if (justReachedTargetFlag) {
            justReachedTargetFlag = false;
            return true;
        }
        return false;
    }

    /** Call once per loop. Computes PIDF output and writes it to both shooters. */
    public void update() {
        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3;

        double currentRpmNow = getCurrentRpm(dt);
        // Low-pass filter on measured RPM (alpha = 1 => no filtering).
        currentRpm = rpmFilterAlpha * currentRpmNow + (1.0 - rpmFilterAlpha) * currentRpm;

        timer.reset();

        double error = targetRpm - currentRpm;

        // Integral with clamping (anti-windup).
        integralSum += error * dt;
        if (integralSum > integralLimit) integralSum = integralLimit;
        if (integralSum < -integralLimit) integralSum = -integralLimit;

        // Derivative with low-pass on error change.
        double rawDeriv = (error - lastError) / dt;
        double deriv = derivativeAlpha * lastDerivativeEstimate + (1.0 - derivativeAlpha) * rawDeriv;
        lastDerivativeEstimate = deriv;

        // Feedforward: fraction of max speed at current battery voltage.
        double voltage = getBatteryVoltage();
        double maxTicksPerSec = (voltage / FF_REFERENCE_VOLTAGE) * FF_REFERENCE_MAX_TICKS_PER_SEC;
        if (maxTicksPerSec < 1e-3) maxTicksPerSec = 1e-3;
        double targetTicksPerSec = (targetRpm * TICKS_PER_REV) / 60.0;
        double ff = (targetTicksPerSec / maxTicksPerSec) * kF;

        // PIDF output, clamped.
        double out = ff + (kP * error) + (kI * integralSum) + (kD * deriv);
        out = Math.max(-1.0, Math.min(1.0, out));

        if (!shooterOn) out = 0.0;

        // Power smoothing (alpha = 1 => no smoothing).
        double smoothedOut = powerSmoothingAlpha * out + (1.0 - powerSmoothingAlpha) * lastAppliedPower;

        try {
            shooterLeft.setPower(smoothedOut);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("Flywheel.power", "left setPower failed: " + e.getMessage());
        }
        try {
            shooterRight.setPower(smoothedOut); // same magnitude; opposite spin comes from direction
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("Flywheel.power", "right setPower failed: " + e.getMessage());
        }

        lastAppliedPower = smoothedOut;
        lastError = error;

        // At-target detection (rising edge sets justReachedTargetFlag).
        boolean atTargetNow = Math.abs(error) <= RPM_TOLERANCE;
        if (atTargetNow && !lastAtTarget) {
            justReachedTargetFlag = true;
        }
        lastAtTarget = atTargetNow;
    }

    private double getBatteryVoltage() {
        try {
            if (voltageSensor != null) {
                double v = voltageSensor.getVoltage();
                if (v > 1e-3) return v;
            }
        } catch (Exception ignored) {
        }
        return 11.0; // fallback
    }

    private double getCurrentRpm(double dtSeconds) {
        // Prefer built-in velocity; fall back to position diff.
        double ticksPerSecond;
        try {
            ticksPerSecond = shooterLeft.getVelocity(); // ticks/sec
            lastPos = shooterLeft.getCurrentPosition();
        } catch (Exception e) {
            int pos = shooterLeft.getCurrentPosition();
            int delta = pos - lastPos;
            lastPos = pos;
            double dt = (dtSeconds <= 0) ? 1e-3 : dtSeconds;
            ticksPerSecond = delta / dt;
        }
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }
}
