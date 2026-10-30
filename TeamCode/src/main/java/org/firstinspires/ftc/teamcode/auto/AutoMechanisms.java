package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.firstinspires.ftc.teamcode.zenith.PollenEstimate;
import org.firstinspires.ftc.teamcode.zenith.ScoreCommand;

import java.util.function.BooleanSupplier;

/**
 * Every mechanism on the robot during AUTO, each as its own small state machine. The auto's main
 * state machine asks for things (collect, volley, spin up); {@link #update()} runs once per loop
 * and does the work.
 *
 * <ul>
 *   <li><b>Intake</b> (both motors, intakeLeft + intakeRight): OFF, COLLECT (taking pollen in; the
 *       pollen estimate counts it), FEED (pushing pollen through the open gate into the shooter;
 *       not counted), OUTTAKE.</li>
 *   <li><b>Pollen shooter + gate</b> (shooterRight, gateRight): IDLE, SPIN_UP (gate shut, the
 *       flower bar lifting clear; waiting for speed or the spin-up timeout, and for the bar to
 *       settle), FIRING (gate open, intake feeding, for count x the feed
 *       time), then IDLE with the flywheel left spinning for the next volley.</li>
 *   <li><b>Nectar shooter + gate</b> (shooterLeft, gateLeft): held OFF and SHUT every loop. AUTO
 *       is pollen only (team rule), so nothing here can open it.</li>
 *   <li><b>Flower bar</b> (flowerLeft + flowerRight): STOWED to start (the robot fits the start box),
 *       DOWN to collect from a flower, and lifted to SHOOTING for every volley (spin-up and
 *       firing) so the shots clear it.</li>
 * </ul>
 *
 * Pollen count: while collecting, the intake encoders' slowdown (IntakeLoadSensor) sets it, so
 * collecting ends as soon as the robot is really full, and {@link #noNewPollenFor} says when nothing
 * more is coming in. Without the sensor it falls back to the time estimate.
 *
 * Timing numbers are shared with the Zenith auto, so there is one place to tune them:
 * {@link ScoreCommand#SPINUP_TIMEOUT_S}, {@link ScoreCommand#FEED_S_PER_PIECE},
 * {@link ScoreCommand#FEED_EXTRA_S} and {@link PollenEstimate#INTAKE_S_PER_PIECE} (all PLACEHOLDER).
 */
public class AutoMechanisms {

    public enum IntakeState { OFF, COLLECT, FEED, OUTTAKE }

    public enum PollenState { IDLE, SPIN_UP, FIRING }

    private final BioHorsRobot robot;
    private final PollenEstimate pollen;
    private final BooleanSupplier atShootingPose;
    private double readyAt = Double.NaN;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime pollenTimer = new ElapsedTime();
    private final ElapsedTime sinceNewPollen = new ElapsedTime();
    private int lastHeld;
    private long readingsAtWait;

    private IntakeState intake = IntakeState.OFF;
    private PollenState pollenShooter = PollenState.IDLE;
    private int volleyCount;
    private int volleysFired;
    private int volleysSkipped;

    /** atShootingPose: whether the robot is at a scoring spot now (ShootingPose); the gate opens only then. */
    public AutoMechanisms(BioHorsRobot robot, int preload, BooleanSupplier atShootingPose) {
        this.robot = robot;
        this.atShootingPose = atShootingPose;
        this.pollen = new PollenEstimate(preload);
        robot.pollenGate.setClosed(true);
        robot.nectarGate.setClosed(true);
        robot.pollenShooter.setShooterOn(false);
        robot.nectarShooter.setShooterOn(false);
        robot.intake.stop();
        robot.flowerBar.setShooting(false);
        robot.flowerBar.setDown(false);
    }

    // ------------------------------------------------------------------ requests

    public void flowerBarDown() {
        robot.flowerBar.setDown(true);
    }

    public void flowerBarStow() {
        robot.flowerBar.setDown(false);
    }

    public void collect() {
        setIntake(IntakeState.COLLECT);
        sinceNewPollen.reset();
        lastHeld = pollen.held();
    }

    /** The robot is at the pickup and waiting for pollen: the no-progress clock starts now. */
    public void waitingForPollen() {
        sinceNewPollen.reset();
        lastHeld = pollen.held();
        readingsAtWait = robot.intakeLoad.readings();
    }

    /**
     * True once, since {@link #waitingForPollen()}, the intake sensor has taken a fresh settled
     * reading and no new pollen has come in for {@code seconds}: there is nothing more to collect
     * here, so stop waiting. Always false without the sensor (the time estimate cannot tell).
     */
    public boolean noNewPollenFor(double seconds) {
        return MechanismConstants.INTAKE_SENSOR_ENABLED && intake == IntakeState.COLLECT
                && robot.intakeLoad.hasReading() && robot.intakeLoad.readings() > readingsAtWait
                && sinceNewPollen.seconds() >= seconds;
    }

    public void intakeOff() {
        setIntake(IntakeState.OFF);
    }

    public void outtake() {
        setIntake(IntakeState.OUTTAKE);
    }

    /** Spins the pollen flywheel up without firing (e.g. while driving to the score spot). */
    public void spinUp() {
        robot.pollenShooter.setTargetRpm(MechanismConstants.AUTO_POLLEN_RPM);
        robot.pollenShooter.setShooterOn(true);
    }

    /**
     * Starts a volley of count pollen. Ignored if one is already running. Skipped altogether when
     * the intake sensor read the robot empty (as MAINHORS26's auto skipped a shot with no balls):
     * feeding nothing would waste about two seconds.
     */
    public void volley(int count) {
        if (pollenShooter != PollenState.IDLE) return;
        if (MechanismConstants.INTAKE_SENSOR_ENABLED && robot.intakeLoad.hasReading() && pollen.held() == 0) {
            volleysSkipped++;
            return;
        }
        volleyCount = count;
        robot.pollenShooter.setTargetRpm(MechanismConstants.AUTO_POLLEN_RPM);
        robot.pollenShooter.setShooterOn(true);
        robot.pollenGate.setClosed(true);
        robot.flowerBar.setShooting(true);   // lift the bar clear first; feeding waits for it
        pollenShooter = PollenState.SPIN_UP;
        pollenTimer.reset();
        readyAt = Double.NaN;
    }

    public boolean volleyDone() {
        return pollenShooter == PollenState.IDLE;
    }

    // ------------------------------------------------------------------ the loop

    /** Once per loop, after the main state machine has made its requests. */
    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        switch (pollenShooter) {
            case IDLE:
                break;
            case SPIN_UP:
                boolean atSpeed = robot.pollenShooter.isAtTarget() || pollenTimer.seconds() >= ScoreCommand.SPINUP_TIMEOUT_S;
                boolean barClear = pollenTimer.seconds() >= MechanismConstants.FLOWER_BAR_SETTLE_S;
                if (atSpeed && barClear) {
                    if (Double.isNaN(readyAt)) readyAt = pollenTimer.seconds();
                    if (atShootingPose.getAsBoolean()) {
                        robot.pollenGate.setClosed(false);
                        setIntake(IntakeState.FEED);
                        pollenShooter = PollenState.FIRING;
                        pollenTimer.reset();
                    } else if (pollenTimer.seconds() - readyAt >= ShootingPose.WAIT_FOR_POSE_S) {
                        // Never reached a scoring spot: the gate stays shut and the pollen stay aboard.
                        robot.pollenGate.setClosed(true);
                        volleysSkipped++;
                        pollenShooter = PollenState.IDLE;
                    }
                }
                break;
            case FIRING:
                if (pollenTimer.seconds() >= volleyCount * ScoreCommand.FEED_S_PER_PIECE + ScoreCommand.FEED_EXTRA_S) {
                    robot.pollenGate.setClosed(true);
                    setIntake(IntakeState.OFF);
                    pollen.launched(volleyCount);
                    robot.intakeLoad.emptied();
                    volleysFired++;
                    pollenShooter = PollenState.IDLE;
                }
                break;
        }

        // The bar lifts clear of the shots for the whole volley.
        robot.flowerBar.setShooting(pollenShooter != PollenState.IDLE);

        // Nectar: off and shut in AUTO, whatever else happens.
        robot.nectarShooter.setShooterOn(false);
        robot.nectarGate.setClosed(true);

        robot.updateMechanisms();

        // While collecting, the count is what the intake feels (or, without the sensor, the time
        // estimate; a long loop, e.g. a pause, is not counted).
        if (intake == IntakeState.COLLECT) {
            if (MechanismConstants.INTAKE_SENSOR_ENABLED) {
                if (robot.intakeLoad.hasReading()) pollen.sensed(robot.intakeLoad.count());
            } else if (dt < 0.25) {
                pollen.intakeRan(dt);
            }
            if (pollen.held() > lastHeld) sinceNewPollen.reset();
            lastHeld = pollen.held();
        }
    }

    /** Everything off: the end of AUTO. */
    public void stopAll() {
        robot.pollenGate.setClosed(true);
        robot.nectarGate.setClosed(true);
        robot.intake.stop();
        robot.pollenShooter.setShooterOn(false);
        robot.nectarShooter.setShooterOn(false);
        robot.flowerBar.setShooting(false);
        robot.flowerBar.setDown(false);
        intake = IntakeState.OFF;
        pollenShooter = PollenState.IDLE;
    }

    private void setIntake(IntakeState state) {
        intake = state;
        switch (state) {
            case COLLECT:
            case FEED:
                robot.intake.intake();
                break;
            case OUTTAKE:
                robot.intake.outtake();
                break;
            default:
                robot.intake.stop();
        }
    }

    // ------------------------------------------------------------------ for telemetry and tests

    public IntakeState intakeState() {
        return intake;
    }

    public PollenState pollenState() {
        return pollenShooter;
    }

    public PollenEstimate pollen() {
        return pollen;
    }

    public int volleysFired() {
        return volleysFired;
    }

    public int volleysSkipped() {
        return volleysSkipped;
    }
}
