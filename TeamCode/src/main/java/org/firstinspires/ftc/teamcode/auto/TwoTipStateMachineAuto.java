package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.firstinspires.ftc.teamcode.aim.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;

import static org.firstinspires.ftc.teamcode.auto.AutoPoses.*;

/**
 * The two-tip pollen auto as a plain Pedro path follower and state machine. No Zenith runtime: the
 * route was drafted and checked in Zenith (autos/two-tip.auto.json), and its points are copied into
 * {@link AutoPoses}. 16 pollen, two tips, then park:
 *
 * <pre>
 *  start: aim, spin up, volley 4 (preload) from where the robot starts
 *        -> one curve to the garden: 4 pollen -> back out, volley 4 from there: south cell tips (tip 1)
 *        -> one curve to flower0 (west wall): 4 -> round the hive's west side -> volley 4 into the north cell
 *        -> flower1 (north wall): 4 -> volley 4: north cell tips (tip 2) -> park
 * </pre>
 *
 * Each {@link State} starts one thing when it is entered (a path, a volley, the intake) and ends
 * when that thing is done or a timeout passes. Every path starts from where the robot really is,
 * like the Zenith draft ("from: current"). Mechanisms are in {@link AutoMechanisms}.
 *
 * Alliance: set with the Choose Alliance OpMode before INIT (RED if never set). The route is RED's;
 * BLUE is its point mirror.
 *
 * A LinearOpMode only because Pedro's Pinpoint reset sleeps during INIT; the loop is the same
 * read, decide, command loop an iterative OpMode would run.
 */
@Autonomous(name = "Two Tip (state machine)", group = "BioHORS", preselectTeleOp = "MainHORS TeleOp")
public class TwoTipStateMachineAuto extends LinearOpMode {

    /** Where the current state is published for the sim tests and telemetry. */
    public static final String STATE_KEY = "biohors.auto.state";

    /** Fraction of top speed for paths with no stated speed (as the Zenith project's default). */
    public static double DEFAULT_SPEED = 0.8;   // SET BY HAND
    public static double GARDEN_SPEED = 0.35;   // SET BY HAND: slow, nose-first into the garden line
    public static double SLIDE_SPEED = 0.4;     // SET BY HAND: slow sideways onto the garden lane, so it stops on time
    public static double FLOWER_SPEED = 0.3;    // SET BY HAND: slow, nose-in to a flower's opening
    public static double GARDEN_TIMEOUT_S = 2.0;      // SET BY HAND: give up waiting for a full hopper
    public static double GARDEN_EXTRA_WAIT_S = 0.8;   // SET BY HAND
    public static double FLOWER_WAIT_S = 1.0;         // SET BY HAND
    public static final int PRELOAD = 4;              // SET BY HAND: the robot starts full (G407 max)
    public static final int VOLLEY = 4;

    public enum State {
        AIM_AND_SPIN_UP,        // a 2 deg turn to face the south cell from the start; the flywheel spins up
        SHOOT_PRELOADS,
        TO_GARDEN,              // one curve, nose-first, ending heading west above the garden
        TURN_SOUTH_AT_GARDEN,
        SIDE_TO_GARDEN,
        COLLECT_GARDEN,         // nose in slowly, intake on; ends at the line or when full
        WAIT_GARDEN_FULL,
        BACK_OUT_OF_GARDEN,
        AIM_FROM_GARDEN,
        SHOOT_GARDEN,           // the south cell reaches 8: tip 1
        TO_FLOWER_WEST,         // one curve, no turn first: it starts the way the volley left the robot facing
        COLLECT_FLOWER_WEST,
        WAIT_FLOWER_WEST,
        BACK_OFF_FLOWER_WEST,
        TURN_NORTH,
        AROUND_WEST_TO_SCORE_NORTH,
        SHOOT_FLOWER_WEST,
        TURN_TO_FLOWER_NORTH,
        COLLECT_FLOWER_NORTH,
        WAIT_FLOWER_NORTH,
        BACK_OFF_FLOWER_NORTH,
        TURN_TO_SCORE_NORTH,
        SHOOT_FLOWER_NORTH,     // the north cell reaches 8: tip 2
        TURN_TO_PARK,
        PARK,
        DONE
    }

    private BioHorsRobot robot;
    private Follower follower;
    private ForesightConfig foresight;
    private AutoMechanisms mech;
    private Alliance alliance = Alliance.RED;
    private State state = State.AIM_AND_SPIN_UP;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initRobot();
        while (opModeInInit()) {
            initLoop();
            telemetry.update();
        }
        if (isStopRequested()) {
            finish();
            return;
        }
        startAuto();
        while (opModeIsActive()) {
            loopOnce();
            telemetry.update();
        }
        finish();
    }

    private void initRobot() {
        Alliance saved = PoseHandoff.savedAlliance();
        alliance = saved == null ? Alliance.RED : saved;
        robot = new BioHorsRobot(hardwareMap, telemetry);
        foresight = Constants.foresightConfig();
        follower = Constants.createFollower(hardwareMap, foresight);
        follower.setPose(p(START));
        mech = new AutoMechanisms(robot, PRELOAD);
        publish();
    }

    private void initLoop() {
        telemetry.addData("Alliance", "%s (change it with the Choose Alliance OpMode)", alliance);
        telemetry.addData("Pinpoint", robot.pinpointReady() ? "READY" : "NOT READY");
        telemetry.addData("Start", "%s", p(START));
    }

    private void startAuto() {
        matchTimer.reset();
        enter(State.AIM_AND_SPIN_UP);
    }

    private void loopOnce() {
        follower.update();
        run();
        mech.update();
        publish();
        telemetry.addData("State", "%s (%.1f s in, %.1f s total)", state, stateTimer.seconds(), matchTimer.seconds());
        telemetry.addData("Pose", "%s", follower.pose());
        telemetry.addData("Pollen", "%d (%s)", mech.pollen().held(),
                MechanismConstants.INTAKE_SENSOR_ENABLED ? "intake sensor" : "time estimate");
        telemetry.addData("Intake speed", "%.0f ticks/s, slowdown %.0f %%", robot.intakeLoad.speedTicksPerS(),
                100 * robot.intakeLoad.slowdown());
        telemetry.addData("Intake / pollen shooter", "%s / %s", mech.intakeState(), mech.pollenState());
    }

    private void finish() {
        if (mech != null) mech.stopAll();
        if (robot != null) robot.stop();
        if (follower != null) {
            Pose pose = follower.pose();
            if (pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())) {
                PoseHandoff.save(alliance, pose.x(), pose.y(), pose.heading());
            }
        }
    }

    // ------------------------------------------------------------------ the state machine

    /** Called every loop: has the current state finished? If so, move on. */
    private void run() {
        switch (state) {
            case SHOOT_PRELOADS:
            case SHOOT_GARDEN:
            case SHOOT_FLOWER_WEST:
            case SHOOT_FLOWER_NORTH:
                if (mech.volleyDone()) next();
                break;

            case COLLECT_GARDEN:
                // A race: the slow path to the line, or a full hopper, or the timeout.
                if (!follower.isBusy() || mech.pollen().full() || stateTimer.seconds() >= GARDEN_TIMEOUT_S) next();
                break;
            case WAIT_GARDEN_FULL:
                // Full, or nothing new coming in (the sensor says so), or out of time: move on.
                if (mech.pollen().full() || mech.noNewPollenFor(MechanismConstants.COLLECT_NO_PROGRESS_S)
                        || stateTimer.seconds() >= GARDEN_EXTRA_WAIT_S) next();
                break;
            case WAIT_FLOWER_WEST:
            case WAIT_FLOWER_NORTH:
                if (mech.pollen().full() || mech.noNewPollenFor(MechanismConstants.COLLECT_NO_PROGRESS_S)
                        || stateTimer.seconds() >= FLOWER_WAIT_S) next();
                break;

            case DONE:
                break;

            default:
                // Every other state is one path.
                if (!follower.isBusy()) next();
        }
    }

    /** What each state starts when the robot enters it. */
    private void enter(State s) {
        state = s;
        stateTimer.reset();
        switch (s) {
            case AIM_AND_SPIN_UP:
                mech.spinUp();
                turn(AIM_FROM_START, FACING_NORTH, SOUTH_CELL_FROM_START);
                break;
            case SHOOT_PRELOADS:
            case SHOOT_GARDEN:
            case SHOOT_FLOWER_WEST:
            case SHOOT_FLOWER_NORTH:
                mech.volley(VOLLEY);
                break;

            case TO_GARDEN:
                go(tangent(Paths.curve(here(), p(TO_GARDEN_C1), p(TO_GARDEN_C2), p(GARDEN_LANE))), 1.0);
                break;
            case TURN_SOUTH_AT_GARDEN:
                turn(TURN_SOUTH_AT_GARDEN, FACING_WEST_NEGATIVE, FACING_SOUTH);
                break;
            case SIDE_TO_GARDEN:
                // A short, slow sideways slide, square to the west wall: tilting the 22 in robot near the
                // wall swings the flower bar's corner into it.
                go(Paths.line(here(), p(GARDEN_APPROACH)).constant(h(FACING_SOUTH)), SLIDE_SPEED);
                break;
            case COLLECT_GARDEN:
                mech.collect();
                // Heading held square to the west wall: the last 0.35 in west is a tiny slide.
                go(Paths.line(here(), p(GARDEN_PICKUP)).constant(h(FACING_SOUTH)), GARDEN_SPEED);
                break;
            case WAIT_GARDEN_FULL:
            case WAIT_FLOWER_WEST:
            case WAIT_FLOWER_NORTH:
                mech.waitingForPollen();   // at the pickup now: the no-progress clock starts here
                break;
            case BACK_OUT_OF_GARDEN:
                mech.intakeOff();
                go(Paths.line(here(), p(GARDEN_TURN)).constant(h(FACING_SOUTH)), DEFAULT_SPEED);
                break;
            case AIM_FROM_GARDEN:
                turn(AIM_FROM_GARDEN, FACING_SOUTH, SOUTH_CELL_FROM_GARDEN);
                break;

            case TO_FLOWER_WEST:
                go(tangent(Paths.curve(here(), p(TO_FLOWER_WEST_C1), p(TO_FLOWER_WEST_C2), p(FLOWER_WEST_APPROACH))), 1.0);
                break;
            case COLLECT_FLOWER_WEST:
                mech.flowerBarDown();
                mech.collect();
                // Heading held square to the wall: a tangent heading would turn a small sideways offset
                // into a yaw that swings the mouth off the flower's opening.
                go(Paths.line(here(), p(FLOWER_WEST_PICKUP)).constant(h(FACING_WEST)), FLOWER_SPEED);
                break;
            case BACK_OFF_FLOWER_WEST:
                mech.intakeOff();
                mech.flowerBarStow();
                go(Paths.line(here(), p(FLOWER_WEST_APPROACH)).reverseTangent(), DEFAULT_SPEED);
                break;
            case TURN_NORTH:
                turn(TURN_NORTH, FACING_WEST, FACING_NORTH);
                break;
            case AROUND_WEST_TO_SCORE_NORTH:
                // Round the WEST side of the hive frame, arriving already facing the north cell.
                go(tangent(Paths.curve(here(), p(AROUND_WEST_C1), p(AROUND_WEST_C2), p(SCORE_NORTH))), 1.0);
                break;

            case TURN_TO_FLOWER_NORTH:
                turn(NORTH_OF_SCORE_NORTH, FACING_NORTH_CELL, FACING_NORTH);
                break;
            case COLLECT_FLOWER_NORTH:
                mech.flowerBarDown();
                mech.collect();
                go(Paths.line(here(), p(FLOWER_NORTH_PICKUP)).constant(h(FACING_NORTH)), FLOWER_SPEED);
                break;
            case BACK_OFF_FLOWER_NORTH:
                mech.intakeOff();
                mech.flowerBarStow();
                go(Paths.line(here(), p(NORTH_OF_SCORE_NORTH)).reverseTangent(), DEFAULT_SPEED);
                break;
            case TURN_TO_SCORE_NORTH:
                turn(SCORE_NORTH, FACING_NORTH, FACING_NORTH_CELL);
                break;

            case TURN_TO_PARK:
                turn(TURN_TO_PARK, FACING_NORTH_CELL, TOWARD_PARK);
                break;
            case PARK:
                mech.stopAll();
                go(tangent(Paths.line(here(), p(PARK))), 1.0);
                break;
            case DONE:
                break;
        }
    }

    private void next() {
        State[] all = State.values();
        enter(all[Math.min(state.ordinal() + 1, all.length - 1)]);
    }

    // ------------------------------------------------------------------ path helpers

    /** Follow a path, capped at a fraction of the robot's top speed. */
    private void go(Path path, double speedFraction) {
        double f = Math.max(0.05, Math.min(1.0, speedFraction));
        follower.follow(path.with(foresight.maxPathSpeed.at(f * Constants.MAX_FORWARD_IN_PER_S)));
    }

    /** A turn on the spot: about an inch to a point, heading swept from one to the other. */
    private void turn(Pose to, double fromRed, double toRed) {
        go(Paths.line(here(), p(to)).linear(h(fromRed), h(toRed)), DEFAULT_SPEED);
    }

    private static Path tangent(Path path) {
        return path.tangent();
    }

    private Pose here() {
        return follower.pose();
    }

    private Pose p(Pose red) {
        return AutoPoses.of(alliance, red);
    }

    private double h(double red) {
        return AutoPoses.heading(alliance, red);
    }

    private void publish() {
        blackboard.put(STATE_KEY, state.name());
    }

    // ------------------------------------------------------------------ for tests

    public State state() {
        return state;
    }

    public AutoMechanisms mechanisms() {
        return mech;
    }
}
