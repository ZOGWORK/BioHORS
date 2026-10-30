package org.firstinspires.ftc.teamcode.zenith;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.function.BooleanSupplier;

import org.firstinspires.ftc.teamcode.auto.ShootingPose;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;

/**
 * Launches {@code count} pollen into the up hive cell. The robot stands still while it runs (it is
 * "stationary" in autos/robot.json), so the follower holds the score pose.
 *
 * Sequence: lift the flower bar clear of the shots and spin the pollen shooter up (or give up
 * waiting after SPINUP_TIMEOUT_S); once both are done (the bar gets FLOWER_BAR_SETTLE_S) and the
 * robot is at a scoring spot (auto/ShootingPose), open the pollen gate and run the intake inward to feed pollen past the gate for count * FEED_S_PER_PIECE,
 * then close the gate and stop the intake. The shooter is left spinning for the next volley. If the
 * robot is not at a scoring spot within ShootingPose.WAIT_FOR_POSE_S of being ready, the volley is
 * skipped with the gate shut: the pollen stay aboard. The spot is checked the whole volley: knocked
 * off it, the gate shuts and feeding pauses until the robot is back (or the volley ends after
 * WAIT_FOR_POSE_S), and only the feeding time really spent counts as pollen launched.
 *
 * ASSUMPTION (NEEDS CONFIRMATION): pollen reaches the shooter by the intake pushing it through the
 * open gate. If the robot feeds differently, this is the one place to change.
 */
public class ScoreCommand extends CommandBase {

    public static double SPINUP_TIMEOUT_S = 2.0;   // PLACEHOLDER
    public static double FEED_S_PER_PIECE = 0.4;   // PLACEHOLDER: NEEDS MEASUREMENT (launch cadence)
    public static double FEED_EXTRA_S = 0.2;       // PLACEHOLDER: last pollen clears the gate

    private final ZenithSubsystems.Launcher launcher;
    private final ZenithSubsystems.Intake intake;
    private final PollenEstimate pollen;
    private final int count;
    private final BooleanSupplier atShootingPose;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean feeding;
    private double readyAt;
    private double fedSeconds;
    private double lastS;
    private double offSince;
    private boolean paused;
    private boolean done;

    public ScoreCommand(ZenithSubsystems.Launcher launcher, ZenithSubsystems.Intake intake,
                        PollenEstimate pollen, int count, BooleanSupplier atShootingPose) {
        this.launcher = launcher;
        this.intake = intake;
        this.pollen = pollen;
        this.count = count;
        this.atShootingPose = atShootingPose;
        addRequirements(launcher, intake);
    }

    @Override
    public void initialize() {
        feeding = false;
        done = false;
        readyAt = Double.NaN;
        fedSeconds = 0;
        offSince = Double.NaN;
        paused = false;
        launcher.closeGate();
        launcher.barClearOfShots(true);
        launcher.spinUp();
        timer.reset();
    }

    @Override
    public void execute() {
        if (!feeding) {
            boolean atSpeed = launcher.atSpeed() || timer.seconds() >= SPINUP_TIMEOUT_S;
            boolean barClear = timer.seconds() >= MechanismConstants.FLOWER_BAR_SETTLE_S;
            if (atSpeed && barClear) {
                if (Double.isNaN(readyAt)) readyAt = timer.seconds();
                if (atShootingPose.getAsBoolean()) {
                    feeding = true;
                    launcher.openGate();
                    intake.feed();
                    timer.reset();
                    lastS = 0;
                } else if (timer.seconds() - readyAt >= ShootingPose.WAIT_FOR_POSE_S) {
                    done = true;   // never reached a scoring spot: keep the gate shut, skip the volley
                }
            }
            return;
        }
        double now = timer.seconds(), dt = now - lastS;
        lastS = now;
        if (atShootingPose.getAsBoolean()) {
            if (paused) {
                paused = false;
                offSince = Double.NaN;
                launcher.openGate();
                intake.feed();
            } else {
                fedSeconds += dt;
            }
        } else {
            if (!paused) {
                // Knocked off the spot mid-volley: shut the gate and stop feeding until it is back.
                paused = true;
                offSince = now;
                launcher.closeGate();
                intake.stop();
            }
            if (now - offSince >= ShootingPose.WAIT_FOR_POSE_S) done = true;
        }
        if (fedSeconds >= fullVolleySeconds()) {
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        launcher.closeGate();
        launcher.barClearOfShots(false);
        intake.stop();
        if (feeding) {
            int fed = ShootingPose.fedPieces(fedSeconds, count, FEED_S_PER_PIECE, fullVolleySeconds());
            pollen.launched(fed);
            if (fed == count) launcher.emptied();
        }
    }

    private double fullVolleySeconds() {
        return count * FEED_S_PER_PIECE + FEED_EXTRA_S;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
