package org.firstinspires.ftc.teamcode.zenith;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;

/**
 * Launches {@code count} pollen into the up hive cell. The robot stands still while it runs (it is
 * "stationary" in autos/robot.json), so the follower holds the score pose.
 *
 * Sequence: lift the flower bar clear of the shots and spin the pollen shooter up (or give up
 * waiting after SPINUP_TIMEOUT_S); once both are done (the bar gets FLOWER_BAR_SETTLE_S), open the
 * pollen gate and run the intake inward to feed pollen past the gate for count * FEED_S_PER_PIECE,
 * then close the gate and stop the intake. The shooter is left spinning for the next volley.
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
    private final ElapsedTime timer = new ElapsedTime();
    private boolean feeding;
    private boolean done;

    public ScoreCommand(ZenithSubsystems.Launcher launcher, ZenithSubsystems.Intake intake,
                        PollenEstimate pollen, int count) {
        this.launcher = launcher;
        this.intake = intake;
        this.pollen = pollen;
        this.count = count;
        addRequirements(launcher, intake);
    }

    @Override
    public void initialize() {
        feeding = false;
        done = false;
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
                feeding = true;
                launcher.openGate();
                intake.feed();
                timer.reset();
            }
            return;
        }
        if (timer.seconds() >= count * FEED_S_PER_PIECE + FEED_EXTRA_S) {
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        launcher.closeGate();
        launcher.barClearOfShots(false);
        intake.stop();
        if (feeding) pollen.launched(count);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
