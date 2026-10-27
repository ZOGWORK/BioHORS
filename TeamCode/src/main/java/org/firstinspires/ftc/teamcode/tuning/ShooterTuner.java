package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Measures each shooter and prints its PIDF, instead of hand-guessing it. A tuning tool, not a
 * match OpMode: run it when a shooter changes.
 *
 * ROBOT ON BLOCKS OR IN A CLEAR AREA, NO POLLEN LOADED. The shooters spin fast.
 *
 * For the pollen shooter, then the nectar shooter (one at a time, so the numbers never mix):
 *   1. Feedforward from a power sweep: run at 0.2..1.0 power, wait for a steady speed, record
 *      speed and battery voltage, fit power-vs-speed. Slope -> max ticks/s (the feedforward),
 *      intercept -> kS (static friction). Normalised to 12 V, and ShooterController scales by
 *      12 V / measured voltage, so the constants hold as the battery drains.
 *   2. P by search: with that feedforward and I = D = 0, step from 70 % to the target RPM and
 *      raise P until the step overshoots or rings (or stops getting faster), then back off 20 %.
 *   3. Hold check: spin-up time from rest, the steady error band, and recovery after a simulated
 *      load dip (power cut for 0.15 s, like a pollen passing through).
 *
 * It holds whatever target RPM MechanismConstants has; it does not pick it (that is a field test).
 * Copy the printed lines into each shooter's "tuned values" block in MechanismConstants.
 */
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    static final double V_REF = 12.0;
    static final double[] SWEEP_POWERS = {0.2, 0.4, 0.6, 0.8, 1.0};
    static final double SETTLE_WINDOW_S = 0.3;
    static final double SETTLE_SPREAD = 0.005;   // steady = spread over the window under 0.5 %
    static final double SETTLE_MIN_S = 0.8;
    static final double SETTLE_MAX_S = 6.0;   // a heavy flywheel spins up slowly; 5 points stay under a minute

    static final double BAND = 0.02;             // "at target" = within 2 %
    static final double MAX_OVERSHOOT = 0.03;
    static final double P_START = 0.0002;
    static final double P_GROWTH = 1.6;
    static final double P_MAX = 0.05;
    static final double P_MARGIN = 0.8;          // back off 20 % from the best P found
    static final double STEP_WINDOW_S = 2.0;
    static final double DIP_S = 0.15;

    private VoltageSensor battery;
    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void runOpMode() {
        try {
            battery = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {
        }
        telemetry.setAutoClear(true);
        telemetry.addLine("SHOOTER TUNER: robot on blocks, NO POLLEN loaded.");
        telemetry.addLine("Press PLAY to tune the pollen shooter, then the nectar shooter.");
        telemetry.update();
        waitForStart();

        List<String> lines = new ArrayList<>();
        tune("Pollen", "POLLEN_SHOOTER", MechanismConstants.POLLEN_SHOOTER_NAME,
                MechanismConstants.POLLEN_SHOOTER_DIRECTION, MechanismConstants.POLLEN_SHOOTER, lines);
        tune("Nectar", "NECTAR_SHOOTER", MechanismConstants.NECTAR_SHOOTER_NAME,
                MechanismConstants.NECTAR_SHOOTER_DIRECTION, MechanismConstants.NECTAR_SHOOTER, lines);

        // Keep the results on screen until STOP.
        telemetry.setAutoClear(true);
        while (opModeIsActive()) {
            telemetry.addLine("DONE. Paste into MechanismConstants (also in the robot log, tag ShooterTuner):");
            for (String l : lines) telemetry.addLine(l);
            telemetry.update();
            sleep(100);
        }
    }

    private void tune(String label, String field, String motorName, DcMotorSimple.Direction direction,
                      ShooterController.Config original, List<String> out) {
        if (!opModeIsActive()) return;
        DcMotorEx motor = (DcMotorEx) hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ---- 1. feedforward from a power sweep ----
        int n = SWEEP_POWERS.length;
        double[] volts = new double[n];
        double[] speed = new double[n];
        for (int i = 0; i < n && opModeIsActive(); i++) {
            motor.setPower(SWEEP_POWERS[i]);
            double[] vs = settle(motor, label + " sweep " + SWEEP_POWERS[i]);
            speed[i] = vs[0];
            volts[i] = vs[1];
        }
        motor.setPower(0);
        if (!opModeIsActive()) return;
        ShooterTuning.Feedforward ff = ShooterTuning.fitFeedforward(SWEEP_POWERS, volts, speed, V_REF);
        double meanV = 0;
        for (double v : volts) meanV += v / n;
        log(String.format(Locale.ROOT, "%s sweep: kS %.4f, max %.0f ticks/s at 12 V, fit r2 %.4f, battery %.2f V",
                label, ff.kS, ff.maxTicksPerSec, ff.r2, meanV));
        spinDown(motor);

        // ---- 2. P by search, I = D = 0 ----
        ShooterController.Config cfg = copy(original);
        cfg.label = label + " (tuning)";
        cfg.ffReferenceVoltage = V_REF;
        cfg.ffReferenceMaxTicksPerSec = ff.maxTicksPerSec;
        cfg.kF = 1.0;
        cfg.kS = ff.kS;
        cfg.kI = 0.0;
        cfg.kD = 0.0;
        double target = original.targetRpm;
        ShooterController shooter = new ShooterController(cfg, motor, telemetry, battery);

        double bestP = 0;
        double bestRise = Double.MAX_VALUE;
        int noGain = 0;
        for (double p = P_START; p <= P_MAX && opModeIsActive(); p *= P_GROWTH) {
            cfg.kP = p;
            shooter.setShooterOn(true);
            shooter.setTargetRpm(0.7 * target);
            runFor(shooter, 2.0, null, null);
            shooter.setTargetRpm(target);
            List<Double> t = new ArrayList<>();
            List<Double> rpm = new ArrayList<>();
            runFor(shooter, STEP_WINDOW_S, t, rpm);
            ShooterTuning.Step step = ShooterTuning.analyzeStep(t, rpm, target, BAND);
            log(String.format(Locale.ROOT, "%s P %.5f: %s", label, p, step));
            if (!step.good(BAND, MAX_OVERSHOOT)) {
                if (bestP > 0) break; // went too far: stop and keep the last good P
                continue;             // not reaching target yet: keep raising P
            }
            if (step.riseTimeS < bestRise * 0.97) {
                noGain = 0;
            } else if (++noGain >= 2) {
                bestP = p;
                break;                // no longer getting faster
            }
            bestP = p;
            bestRise = Math.min(bestRise, step.riseTimeS);
        }
        shooter.setShooterOn(false);
        runFor(shooter, 0.1, null, null);
        spinDown(motor);
        if (!opModeIsActive()) return;
        double chosenP = bestP * P_MARGIN;
        if (bestP == 0) log(label + ": NO P held the target cleanly; check the shooter and re-run. kP left at 0.");

        // ---- 3. hold check with the chosen numbers ----
        cfg.kP = chosenP;
        shooter.setTargetRpm(target);
        shooter.setShooterOn(true);
        List<Double> t = new ArrayList<>();
        List<Double> rpm = new ArrayList<>();
        runFor(shooter, 3.0, t, rpm);
        double spinUp = ShooterTuning.analyzeStep(t, rpm, target, BAND).riseTimeS;
        t.clear();
        rpm.clear();
        runFor(shooter, 1.5, t, rpm);
        double band = ShooterTuning.analyzeStep(t, rpm, target, BAND).steadyError;
        // Simulated load dip: power cut for DIP_S, as a pollen passing through drags the wheel.
        shooter.setShooterOn(false);
        List<Double> dipRpm = new ArrayList<>();
        runFor(shooter, DIP_S, null, dipRpm);
        double dipLow = target;
        for (double r : dipRpm) dipLow = Math.min(dipLow, r);
        shooter.setShooterOn(true);
        t.clear();
        rpm.clear();
        runFor(shooter, 2.0, t, rpm);
        double recovery = ShooterTuning.analyzeStep(t, rpm, target, BAND).riseTimeS;
        shooter.setShooterOn(false);
        runFor(shooter, 0.1, null, null);
        spinDown(motor);

        log(String.format(Locale.ROOT,
                "%s hold check at %.0f rpm: spin-up %.2f s, steady within %.1f %%, dip to %.0f rpm recovered in %.2f s",
                label, target, spinUp, 100 * band, dipLow, recovery));
        out.add(String.format(Locale.ROOT, "// %s: MEASURED by Shooter Tuner, sweep at %.2f V (fit r2 %.3f); "
                        + "hold: spin-up %.2f s, steady %.1f %%, dip recovery %.2f s",
                label, meanV, ff.r2, spinUp, 100 * band, recovery));
        out.add(String.format(Locale.ROOT, "%s.ffReferenceVoltage = %.1f;", field, V_REF));
        out.add(String.format(Locale.ROOT, "%s.ffReferenceMaxTicksPerSec = %.1f;", field, ff.maxTicksPerSec));
        out.add(String.format(Locale.ROOT, "%s.kF = 1.0;", field));
        out.add(String.format(Locale.ROOT, "%s.kS = %.4f;", field, ff.kS));
        out.add(String.format(Locale.ROOT, "%s.kP = %.6f;", field, chosenP));
        out.add(String.format(Locale.ROOT, "%s.kI = 0.0;", field));
        out.add(String.format(Locale.ROOT, "%s.kD = 0.0;", field));
        for (int i = out.size() - 8; i < out.size(); i++) RobotLog.ii("ShooterTuner", out.get(i));
    }

    /** Waits for a steady speed at the current power; returns {mean ticks/s, mean volts}. */
    private double[] settle(DcMotorEx motor, String what) {
        List<double[]> window = new ArrayList<>(); // {time, speed, volts}
        clock.reset();
        while (opModeIsActive()) {
            double now = clock.seconds();
            window.add(new double[] {now, motor.getVelocity(), volts()});
            while (!window.isEmpty() && window.get(0)[0] < now - SETTLE_WINDOW_S) window.remove(0);
            double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE, mean = 0, v = 0;
            for (double[] w : window) {
                lo = Math.min(lo, w[1]);
                hi = Math.max(hi, w[1]);
                mean += w[1] / window.size();
                v += w[2] / window.size();
            }
            boolean steady = now >= SETTLE_MIN_S && hi - lo <= Math.max(SETTLE_SPREAD * Math.abs(mean), 10.0);
            telemetry.addData("Tuning", "%s: %.0f ticks/s", what, mean);
            telemetry.update();
            if (steady || now >= SETTLE_MAX_S) return new double[] {mean, v};
            idle();
        }
        return new double[] {0, V_REF};
    }

    /** Runs the controller for a while, recording (seconds, rpm) if lists are given. */
    private void runFor(ShooterController shooter, double seconds, List<Double> t, List<Double> rpm) {
        clock.reset();
        while (opModeIsActive() && clock.seconds() < seconds) {
            shooter.update();
            if (t != null) t.add(clock.seconds());
            if (rpm != null) rpm.add(shooter.getCurrentRpm());
            telemetry.addData("Tuning", "%s: %.0f / %.0f rpm", shooter.getLabel(),
                    shooter.getCurrentRpm(), shooter.getTargetRpm());
            telemetry.update();
            idle();
        }
    }

    private void spinDown(DcMotorEx motor) {
        motor.setPower(0);
        clock.reset();
        while (opModeIsActive() && clock.seconds() < 6.0 && Math.abs(motor.getVelocity()) > 50) idle();
    }

    private double volts() {
        try {
            if (battery != null) {
                double v = battery.getVoltage();
                if (v > 1) return v;
            }
        } catch (Exception ignored) {
        }
        return V_REF;
    }

    private void log(String line) {
        RobotLog.ii("ShooterTuner", line);
        telemetry.log().add(line);
    }

    private static ShooterController.Config copy(ShooterController.Config c) {
        ShooterController.Config d = new ShooterController.Config();
        d.label = c.label;
        d.ticksPerRev = c.ticksPerRev;
        d.kP = c.kP;
        d.kI = c.kI;
        d.kD = c.kD;
        d.kF = c.kF;
        d.kS = c.kS;
        d.integralLimit = c.integralLimit;
        d.derivativeAlpha = c.derivativeAlpha;
        d.rpmFilterAlpha = c.rpmFilterAlpha;
        d.powerSmoothingAlpha = c.powerSmoothingAlpha;
        d.targetRpm = c.targetRpm;
        d.rpmTolerance = c.rpmTolerance;
        d.ffReferenceVoltage = c.ffReferenceVoltage;
        d.ffReferenceMaxTicksPerSec = c.ffReferenceMaxTicksPerSec;
        return d;
    }
}
