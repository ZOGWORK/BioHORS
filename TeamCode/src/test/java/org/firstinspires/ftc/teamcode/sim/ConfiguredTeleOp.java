package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.aim.AimController;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.firstinspires.ftc.teamcode.teleop.MainHorsTeleOp;

/**
 * The MainHORS teleop, unchanged, with a few of its tuning statics set by the test first.
 *
 * SimLoop builds this class inside its sim-clock class loader (OpModeRunner.forTeamClass), which
 * has its own copy of every team class, so a test cannot set AimController's statics directly. It
 * puts the values on OpMode.blackboard instead; this sets them in the loader's copy and puts them
 * back afterwards. A key that is absent leaves that value as it is.
 */
public final class ConfiguredTeleOp extends MainHorsTeleOp {

    static final String CLASS = "org.firstinspires.ftc.teamcode.sim.ConfiguredTeleOp";
    static final String BALL_SPEED = "sim.teleop.ballGroundSpeedInPerS";
    static final String POLLEN_TABLE_DISTANCE = "sim.teleop.pollenTableDistanceIn";
    static final String POLLEN_TABLE_RPM = "sim.teleop.pollenTableRpm";
    static final String BAR_AUTO_STOW = "sim.teleop.barAutoStow";
    static final String SHOOTER_IDLE = "sim.teleop.shooterIdle";

    @Override
    public void runOpMode() {
        double savedBall = AimController.BALL_GROUND_SPEED_IN_PER_S;
        double[] savedDistance = MechanismConstants.POLLEN_SPEED_TABLE_DISTANCE_IN;
        double[] savedRpm = MechanismConstants.POLLEN_SPEED_TABLE_RPM;
        boolean savedAutoStow = BAR_AUTO_STOW_ENABLED;
        boolean savedIdle = MechanismConstants.SHOOTER_IDLE_ENABLED;
        try {
            if (blackboard.containsKey(SHOOTER_IDLE)) {
                MechanismConstants.SHOOTER_IDLE_ENABLED = (Boolean) blackboard.get(SHOOTER_IDLE);
            }
            if (blackboard.containsKey(BAR_AUTO_STOW)) BAR_AUTO_STOW_ENABLED = (Boolean) blackboard.get(BAR_AUTO_STOW);
            if (blackboard.containsKey(BALL_SPEED)) {
                AimController.BALL_GROUND_SPEED_IN_PER_S = (Double) blackboard.get(BALL_SPEED);
            }
            if (blackboard.containsKey(POLLEN_TABLE_DISTANCE)) {
                MechanismConstants.POLLEN_SPEED_TABLE_DISTANCE_IN = (double[]) blackboard.get(POLLEN_TABLE_DISTANCE);
                MechanismConstants.POLLEN_SPEED_TABLE_RPM = (double[]) blackboard.get(POLLEN_TABLE_RPM);
            }
            super.runOpMode();
        } finally {
            AimController.BALL_GROUND_SPEED_IN_PER_S = savedBall;
            MechanismConstants.POLLEN_SPEED_TABLE_DISTANCE_IN = savedDistance;
            MechanismConstants.POLLEN_SPEED_TABLE_RPM = savedRpm;
            BAR_AUTO_STOW_ENABLED = savedAutoStow;
            MechanismConstants.SHOOTER_IDLE_ENABLED = savedIdle;
        }
    }
}
