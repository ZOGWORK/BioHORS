package org.firstinspires.ftc.teamcode.zenith;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.firstinspires.ftc.teamcode.aim.PoseHandoff;
import org.firstinspires.ftc.teamcode.auto.ShootingPose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.horizon36596.zenith.solverslib.AutoFromFile;
import org.horizon36596.zenith.solverslib.ZenithRobot;

/**
 * This robot as the Zenith runtime sees it. zenith.json names this class as deploy.robotClass, and
 * every OpMode `zenith deploy` writes builds one.
 *
 * Alliance: the auto file's own. Each alliance has its own file and OpMode: "Two tip" is
 * autos/two-tip.auto.json (RED), "Two tip (BLUE)" is autos/two-tip-blue.auto.json, written from the
 * RED one by tools/mirror_auto.mjs with Zenith's mirror. Choosing the OpMode chooses the alliance;
 * the "Choose Alliance" blackboard value is not read here, so a value left over from an earlier
 * match can never run the path on the wrong side.
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
    private ZenithSubsystems.FlowerBar flowerBar;
    private PollenEstimate pollen;
    private org.firstinspires.ftc.teamcode.auto.PinpointWatchdog watchdog;
    private Alliance alliance = Alliance.RED;

    @Override
    public void init(AutoFromFile opMode) {
        alliance = "BLUE".equals(opMode.spec().alliance) ? Alliance.BLUE : Alliance.RED;

        hardware = new BioHorsRobot(opMode.hardwareMap, opMode.telemetry);
        foresight = Constants.foresightConfig();
        drive = new ZenithSubsystems.Drive(Constants.createFollower(opMode.hardwareMap, foresight), foresight);
        pollen = new PollenEstimate(PollenEstimate.CAPACITY); // preload: 4 pollen
        intake = new ZenithSubsystems.Intake(hardware, pollen);
        launcher = new ZenithSubsystems.Launcher(hardware);
        flowerBar = new ZenithSubsystems.FlowerBar(hardware);
        watchdog = new org.firstinspires.ftc.teamcode.auto.PinpointWatchdog(hardware::pinpointReady);
        drive.watch(watchdog, hardware.drive::stop);
    }

    @Override
    public void registerCommands(AutoFromFile opMode) {
        // The gate opens only at a scoring spot (auto/ShootingPose), wherever the follower really is.
        BioHorsCommands.register(intake, launcher, flowerBar, pollen,
                () -> !watchdog.tripped() && ShootingPose.at(alliance, drive.follower().pose()));
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
        if (watchdog.tripped()) opMode.telemetry.addData("AUTO HALTED", watchdog.reason());
        opMode.telemetry.addData("Pollen (estimate)", pollen.held());
        if (launcher != null && launcher.spinUpFailures() > 0) {
            opMode.telemetry.addData("SHOOTER", "never reached speed: %d volley(s) skipped, pollen kept",
                    launcher.spinUpFailures());
        }
    }

    @Override
    public void onStart(AutoFromFile opMode) {
        watchdog.start(); // the never-READY timer counts from PLAY
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
