package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * One shooter: a single encoder motor under a custom PIDF velocity loop.
 *
 * The robot has TWO of these, fully independent -- one for pollen, one for
 * nectar -- each with its own target RPM and gains. The gains live in a
 * {@link Config} (see MechanismConstants.POLLEN_SHOOTER / NECTAR_SHOOTER),
 * and this controller reads them every loop, so live edits take effect
 * without a restart.
 *
 * Spin direction is set on the motor in the OpMode, not here.
 */
public class ShooterController {

    /**
     * Per-shooter tuning values. Every value has a provenance label where
     * it is assigned in MechanismConstants; nothing here is measured yet.
     */
    public static class Config {
        public String label;

        /** Encoder ticks per FLYWHEEL revolution (motor ticks x gear ratio). */
        public double ticksPerRev;

        public double kP;
        public double kI;
        public double kD;
        public double kF;
        /**
         * Static-friction power: the power it takes just to keep the flywheel turning, added while
         * the target is above zero. Voltage-compensated like the feedforward. 0 = off.
         */
        public double kS;
        public double integralLimit;
        public double derivativeAlpha;
        public double rpmFilterAlpha;
        public double powerSmoothingAlpha;

        public double targetRpm;
        public double rpmTolerance;

        /**
         * Feedforward reference: motor-encoder ticks/sec at full power at the
         * reference voltage. The encoder is on the motor, so this does not
         * change with an external gear ratio (only ticksPerRev does).
         */
        public double ffReferenceVoltage;
        public double ffReferenceMaxTicksPerSec;
    }

    private final Config config;
    private final DcMotorEx motor;
    private final Telemetry telemetry; // nullable
    private final VoltageSensor voltageSensor; // nullable
    private final ElapsedTime timer = new ElapsedTime();

    // --- Internal state ---
    private double targetRpm;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastDerivativeEstimate = 0.0;
    private int lastPos = 0;
    private double currentRpm = 0.0;
    private double lastAppliedPower = 0.0;

    private boolean shooterOn = false;
    private boolean lastAtTarget = false;
    private boolean justReachedTargetFlag = false;

    public ShooterController(Config config, DcMotor motor,
                             Telemetry telemetry, VoltageSensor voltageSensor) {
        if (!(motor instanceof DcMotorEx)) {
            throw new IllegalArgumentException(config.label + " shooter must be a DcMotorEx");
        }
        this.config = config;
        this.motor = (DcMotorEx) motor;
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;
        this.targetRpm = config.targetRpm;

        try {
            // Our own PIDF does the velocity control, so the motor runs open
            // loop; getVelocity() still reads the encoder.
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData(config.label + ".init", "config failed: " + e.getMessage());
        }

        timer.reset();
        lastPos = this.motor.getCurrentPosition();
    }

    public String getLabel() {
        return config.label;
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

    public boolean isAtTarget() {
        return Math.abs(targetRpm - currentRpm) <= config.rpmTolerance;
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

    /** Call once per loop. Computes PIDF output and writes it to the motor. */
    public void update() {
        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3;

        double currentRpmNow = readRpm(dt);
        // Low-pass filter on measured RPM (alpha = 1 => no filtering).
        currentRpm = config.rpmFilterAlpha * currentRpmNow + (1.0 - config.rpmFilterAlpha) * currentRpm;

        timer.reset();

        double error = targetRpm - currentRpm;

        // Integral with clamping (anti-windup).
        integralSum += error * dt;
        if (integralSum > config.integralLimit) integralSum = config.integralLimit;
        if (integralSum < -config.integralLimit) integralSum = -config.integralLimit;

        // Derivative with low-pass on error change.
        double rawDeriv = (error - lastError) / dt;
        double deriv = config.derivativeAlpha * lastDerivativeEstimate + (1.0 - config.derivativeAlpha) * rawDeriv;
        lastDerivativeEstimate = deriv;

        // Feedforward: fraction of max speed at current battery voltage.
        double voltage = getBatteryVoltage();
        double maxTicksPerSec = (voltage / config.ffReferenceVoltage) * config.ffReferenceMaxTicksPerSec;
        if (maxTicksPerSec < 1e-3) maxTicksPerSec = 1e-3;
        double targetTicksPerSec = (targetRpm * config.ticksPerRev) / 60.0;
        double ff = (targetTicksPerSec / maxTicksPerSec) * config.kF;
        if (targetRpm > 0) ff += config.kS * (config.ffReferenceVoltage / Math.max(voltage, 1e-3));

        // PIDF output, clamped.
        double out = ff + (config.kP * error) + (config.kI * integralSum) + (config.kD * deriv);
        out = Math.max(-1.0, Math.min(1.0, out));

        if (!shooterOn) out = 0.0;

        // Power smoothing (alpha = 1 => no smoothing).
        double smoothedOut = config.powerSmoothingAlpha * out + (1.0 - config.powerSmoothingAlpha) * lastAppliedPower;

        try {
            motor.setPower(smoothedOut);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData(config.label + ".power", "setPower failed: " + e.getMessage());
        }

        lastAppliedPower = smoothedOut;
        lastError = error;

        // At-target detection (rising edge sets justReachedTargetFlag).
        boolean atTargetNow = Math.abs(error) <= config.rpmTolerance;
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

    private double readRpm(double dtSeconds) {
        // Prefer built-in velocity; fall back to position diff.
        double ticksPerSecond;
        try {
            ticksPerSecond = motor.getVelocity(); // ticks/sec
            lastPos = motor.getCurrentPosition();
        } catch (Exception e) {
            int pos = motor.getCurrentPosition();
            int delta = pos - lastPos;
            lastPos = pos;
            double dt = (dtSeconds <= 0) ? 1e-3 : dtSeconds;
            ticksPerSecond = delta / dt;
        }
        return (ticksPerSecond * 60.0) / config.ticksPerRev;
    }
}
