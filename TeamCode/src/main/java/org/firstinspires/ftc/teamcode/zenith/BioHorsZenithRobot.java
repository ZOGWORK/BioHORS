package org.firstinspires.ftc.teamcode.zenith;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.firstinspires.ftc.teamcode.aim.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.horizon36596.zenith.solverslib.AutoFromFile;
import org.horizon36596.zenith.solverslib.ZenithRobot;

/**
 * This robot as the Zenith runtime sees it. zenith.json names this class as deploy.robotClass, and
 * every OpMode `zenith deploy` writes builds one.
 *
 * Alliance: set before INIT with the "Choose Alliance" OpMode (stored on the SDK blackboard);
 * RED if it was never set. The auto files are written for RED and mirrored for BLUE.
 *
 * When the auto stops, the alliance and the final pose go on the blackboard for the teleop's aim.
 */
public class BioHorsZenithRobot implements ZenithRobot {

    /** Same as autos/robot.json kinematics.defaultPathSpeedFraction. PLACEHOLDER. */
    static final double DEFAULT_SPEED_FRACTION = 0.8;

    private BioHorsRobot hardware;
    private ForesightConfig foresight;
    private ZenithSubsystems.Drive drive;
    private ZenithSubsystems.Intake intake;
    private ZenithSubsystems.Launcher launcher;
    private PollenEstimate pollen;
    private Alliance alliance = Alliance.RED;

    @Override
    public void init(AutoFromFile opMode) {
        Alliance saved = PoseHandoff.savedAlliance();
        alliance = saved == null ? Alliance.RED : saved;

        hardware = new BioHorsRobot(opMode.hardwareMap, opMode.telemetry);
        foresight = Constants.foresightConfig();
        drive = new ZenithSubsystems.Drive(Constants.createFollower(opMode.hardwareMap, foresight), foresight);
        pollen = new PollenEstimate(PollenEstimate.CAPACITY); // preload: 4 pollen
        intake = new ZenithSubsystems.Intake(hardware, pollen);
        launcher = new ZenithSubsystems.Launcher(hardware);
    }

    @Override
    public void registerCommands(AutoFromFile opMode) {
        BioHorsCommands.register(intake, launcher, pollen);
    }

    @Override
    public Follower follower() {
        return drive.follower();
    }

    @Override
    public Subsystem drive() {
        return drive;
    }

    @Override
    public String runningAlliance() {
        return alliance.name();
    }

    /** BIOBUZZ is point-symmetric about the field centre: the other alliance is a half turn away. */
    @Override
    public Pose2d mirror(Pose2d pose) {
        return new Pose2d(-pose.getX(), -pose.getY(), new Rotation2d(pose.getHeading() + Math.PI));
    }

    @Override
    public double defaultSpeedFraction() {
        return DEFAULT_SPEED_FRACTION;
    }

    @Override
    public Path atSpeed(Path path, double speedFraction) {
        return drive.atSpeed(path, speedFraction);
    }

    @Override
    public void initLoop(AutoFromFile opMode) {
        opMode.telemetry.addData("Alliance", "%s (change it with the Choose Alliance OpMode)", alliance);
        opMode.telemetry.addData("Pinpoint", hardware.pinpointReady() ? "READY" : "NOT READY");
    }

    @Override
    public void afterLoop(AutoFromFile opMode) {
        opMode.telemetry.addData("Pollen (estimate)", pollen.held());
    }

    @Override
    public void onStop(AutoFromFile opMode) {
        if (hardware != null) hardware.stop();
        if (drive != null) {
            com.pedropathing.math.Pose p = drive.follower().pose();
            if (p != null && Double.isFinite(p.x()) && Double.isFinite(p.y())) {
                PoseHandoff.save(alliance, p.x(), p.y(), p.heading());
            }
        }
    }

    /** The robot's own pollen estimate (for tests and telemetry). */
    public PollenEstimate pollenEstimate() {
        return pollen;
    }
}
