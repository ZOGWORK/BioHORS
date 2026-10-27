package org.firstinspires.ftc.teamcode.zenith;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.horizon36596.zenith.solverslib.NamedCommands;

/**
 * The only verbs and end conditions a Zenith auto may use on this robot. Same names, same
 * arguments, as "commands" and "conditions" in autos/robot.json.
 *
 *   intakeOn      command    runs the intake inward and keeps it running
 *   intakeOff     command    stops the intake
 *   spinUp        command    brings the pollen shooter up to speed (ends at speed)
 *   score         command    launches `count` pollen; the robot stands still while it runs
 *                            (the flower bar lifts slightly while it does)
 *   flowerBarDown command    puts the flower bar down to collect from a flower
 *   flowerBarStow command    folds the flower bar up
 *   holdingPiece  condition  at least one pollen aboard   (estimate: see PollenEstimate)
 *   hopperFull    condition  4 pollen aboard, the maximum (estimate)
 *   hopperEmpty   condition  no pollen aboard              (estimate)
 */
public final class BioHorsCommands {

    public static final String INTAKE_ON = "intakeOn";
    public static final String INTAKE_OFF = "intakeOff";
    public static final String SPIN_UP = "spinUp";
    public static final String SCORE = "score";
    public static final String FLOWER_BAR_DOWN = "flowerBarDown";
    public static final String FLOWER_BAR_STOW = "flowerBarStow";
    public static final String HOLDING_PIECE = "holdingPiece";
    public static final String HOPPER_FULL = "hopperFull";
    public static final String HOPPER_EMPTY = "hopperEmpty";

    /** Give up waiting for the shooter after this long, seconds. PLACEHOLDER. */
    public static double SPIN_UP_TIMEOUT_S = 2.0;

    private BioHorsCommands() {
    }

    public static void register(final ZenithSubsystems.Intake intake,
                                final ZenithSubsystems.Launcher launcher,
                                final ZenithSubsystems.FlowerBar flowerBar,
                                final PollenEstimate pollen) {
        NamedCommands.register(FLOWER_BAR_DOWN, (args, ctx) -> new InstantCommand(flowerBar::down, flowerBar));
        NamedCommands.register(FLOWER_BAR_STOW, (args, ctx) -> new InstantCommand(flowerBar::stow, flowerBar));
        NamedCommands.register(INTAKE_ON, (args, ctx) -> new InstantCommand(intake::in, intake));
        NamedCommands.register(INTAKE_OFF, (args, ctx) -> new InstantCommand(intake::stop, intake));
        NamedCommands.register(SPIN_UP, (args, ctx) -> new SpinUp(launcher));
        NamedCommands.register(SCORE, (args, ctx) ->
                new ScoreCommand(launcher, intake, pollen, NamedCommands.argInt(args, SCORE, "count")));

        NamedCommands.registerCondition(HOLDING_PIECE, () -> !pollen.empty());
        NamedCommands.registerCondition(HOPPER_FULL, pollen::full);
        NamedCommands.registerCondition(HOPPER_EMPTY, pollen::empty);
    }

    /** Turns the pollen shooter on and ends once it is at speed (or after SPIN_UP_TIMEOUT_S). */
    static final class SpinUp extends CommandBase {
        private final ZenithSubsystems.Launcher launcher;
        private final ElapsedTime timer = new ElapsedTime();

        SpinUp(ZenithSubsystems.Launcher launcher) {
            this.launcher = launcher;
            addRequirements(launcher);
        }

        @Override
        public void initialize() {
            launcher.spinUp();
            timer.reset();
        }

        @Override
        public boolean isFinished() {
            return launcher.atSpeed() || timer.seconds() >= SPIN_UP_TIMEOUT_S;
        }
    }
}
