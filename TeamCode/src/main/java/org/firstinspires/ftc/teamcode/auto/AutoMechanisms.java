package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.firstinspires.ftc.teamcode.zenith.PollenEstimate;
import org.firstinspires.ftc.teamcode.zenith.ScoreCommand;

/**
 * Every mechanism on the robot during AUTO, each as its own small state machine. The auto's main
 * state machine asks for things (collect, volley, spin up); {@link #update()} runs once per loop
 * and does the work.
 *
 * <ul>
 *   <li><b>Intake</b> (both motors, intakeLeft + intakeRight): OFF, COLLECT (taking pollen in; the
 *       pollen estimate counts it), FEED (pushing pollen through the open gate into the shooter;
 *       not counted), OUTTAKE.</li>
 *   <li><b>Pollen shooter + gate</b> (shooterRight, gateRight): IDLE, SPIN_UP (gate shut, waiting
 *       for speed or the spin-up timeout), FIRING (gate open, intake feeding, for count x the feed
 *       time), then IDLE with the flywheel left spinning for the next volley.</li>
 *   <li><b>Nectar shooter + gate</b> (shooterLeft, gateLeft): held OFF and SHUT every loop. AUTO
 *       is pollen only (team rule), so nothing here can open it.</li>
 *   <li><b>Flower bar</b> (flowerLeft + flowerRight): STOWED to start (the robot fits the start box),
 *       DOWN to collect from a flower, and lifted to SHOOTING for every volley (spin-up and
 *       firing) so the shots clear it.</li>
 * </ul>
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
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime pollenTimer = new ElapsedTime();

    private IntakeState intake = IntakeState.OFF;
    private PollenState pollenShooter = PollenState.IDLE;
    private int volleyCount;
    private int volleysFired;

    public AutoMechanisms(BioHorsRobot robot, int preload) {
        this.robot = robot;
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
    }

    public void intakeOff() {
        setIntake(IntakeState.OFF);
    }

    public void outtake() {
        setIntake(IntakeState.OUTTAKE);
    }

    /** Spins the pollen flywheel up without firing (e.g. while driving to the score spot). */
    public void spinUp() {
        robot.pollenShooter.setShooterOn(true);
    }

    /** Starts a volley of count pollen. Ignored if one is already running. */
    public void volley(int count) {
        if (pollenShooter != PollenState.IDLE) return;
        volleyCount = count;
        robot.pollenShooter.setShooterOn(true);
        robot.pollenGate.setClosed(true);
        pollenShooter = PollenState.SPIN_UP;
        pollenTimer.reset();
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
                if (robot.pollenShooter.isAtTarget() || pollenTimer.seconds() >= ScoreCommand.SPINUP_TIMEOUT_S) {
                    robot.pollenGate.setClosed(false);
                    setIntake(IntakeState.FEED);
                    pollenShooter = PollenState.FIRING;
                    pollenTimer.reset();
                }
                break;
            case FIRING:
                if (pollenTimer.seconds() >= volleyCount * ScoreCommand.FEED_S_PER_PIECE + ScoreCommand.FEED_EXTRA_S) {
                    robot.pollenGate.setClosed(true);
                    setIntake(IntakeState.OFF);
                    pollen.launched(volleyCount);
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

        // The pollen estimate counts collecting only (a long loop, e.g. a pause, is not counted).
        if (intake == IntakeState.COLLECT && dt < 0.25) pollen.intakeRan(dt);

        robot.updateMechanisms();
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
}
