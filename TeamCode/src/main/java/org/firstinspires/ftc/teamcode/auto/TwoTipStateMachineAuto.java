package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
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
 * Zenith's starter example auto (examples/starter collect-and-score), tuned to this robot, as a
 * plain Pedro path follower and state machine. No Zenith runtime: the route was tuned and checked
 * in Zenith (autos/two-tip.auto.json), and its points are copied into {@link AutoPoses}. 12 pollen,
 * two tips:
 *
 * <pre>
 *  start -> the south score spot: volley 4 preloads (the 3 staged nectar mean they tip the hive)
 *        -> round the west of the hive to flower1 (north wall), bar down: 4 -> volley 4 north
 *        -> back round to the garden lane: sweep west, intake toward the red alliance
 *           wall, taking the 4 garden pollen in a line -> round to the north spot: volley 4: tip 2
 * </pre>
 *
 * Each {@link State} starts one thing when it is entered (a path, a volley, the intake) and ends
 * when that thing is done or a timeout passes. Every path starts from where the robot really is,
 * like the Zenith draft ("from: current"), and the headings follow the draft, piece by piece where
 * it changes mode along a path. Mechanisms are in {@link AutoMechanisms}.
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
    public static double PICKUP_SPEED = 0.3;    // the starter's: slow into the flower
    public static double GARDEN_SWEEP_SPEED = 0.2;   // SET BY HAND (team): slower along the garden row, so every pollen goes in
    public static double SLIDE_SPEED = 0.3;     // SET BY HAND: slow sideways onto the garden lane, so it stops on time
    public static double OFF_LANE_SPEED = 0.6;  // SET BY HAND: up off the garden lane, sideways and back
    public static double SWEEP_TIMEOUT_S = 3.0;       // SET BY HAND: a stuck sweep gives up after this
    public static double FLOWER_WAIT_S = 1.0;         // the starter's
    public static double GARDEN_WAIT_S = 0.8;         // SET BY HAND
    public static double SPIN_UP_AFTER_S = 0.5;       // the starter's: spin up 0.5 s into the drive back
    public static final int PRELOAD = 4;              // SET BY HAND: the robot starts full (G407 max)
    public static final int VOLLEY = 4;

    public enum State {
        DRIVE_OUT,              // start -> south score spot
        SHOOT_PRELOADS,         // tip 1 (3 nectar staged in the south cell)
        TO_OTHER_SIDE,          // round the west of the hive to below flower1
        COLLECT_FLOWER,         // bar down, intake on, slowly in; ends at the opening or when full
        WAIT_FLOWER_FULL,
        TO_SCORE_NORTH,         // the starter's leg: backs off the flower facing it, bar up, turns round
        SHOOT_FLOWER,
        TO_GARDEN,              // back round the west of the hive, ending heading west above the lane
        SLIDE_ONTO_LANE,        // down onto the lane, square to the south wall
        SWEEP_GARDEN,           // intake on, west along the row (bar stowed): the pollen come in a line
        WAIT_GARDEN_FULL,
        OFF_GARDEN_LANE,        // up off the lane still facing west
        DRIVE_BACK,             // round to the north score spot, spinning up on the way
        SHOOT_GARDEN,           // tip 2
        PARK,                   // west to the park spot, partly in RED's loading zone
        DONE
    }

    private BioHorsRobot robot;
    private Follower follower;
    private ForesightConfig foresight;
    private AutoMechanisms mech;
    private Alliance alliance = Alliance.RED;
    private State state = State.DRIVE_OUT;
    private boolean spunUpOnTheWay;
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
        enter(State.DRIVE_OUT);
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
            case SHOOT_FLOWER:
            case SHOOT_GARDEN:
                if (mech.volleyDone()) next();
                break;

            case COLLECT_FLOWER:
                // The starter's end condition: at the opening, or full.
                if (!follower.isBusy() || mech.pollen().full()) next();
                break;
            case WAIT_FLOWER_FULL:
                if (mech.pollen().full() || mech.noNewPollenFor(MechanismConstants.COLLECT_NO_PROGRESS_S)
                        || stateTimer.seconds() >= FLOWER_WAIT_S) next();
                break;
            case SWEEP_GARDEN:
                // The sweep always runs to its end: cut short when full, the robot rolls on toward
                // the west wall. The timeout only guards a stuck path.
                if (!follower.isBusy() || stateTimer.seconds() >= SWEEP_TIMEOUT_S) next();
                break;
            case WAIT_GARDEN_FULL:
                if (mech.pollen().full() || mech.noNewPollenFor(MechanismConstants.COLLECT_NO_PROGRESS_S)
                        || stateTimer.seconds() >= GARDEN_WAIT_S) next();
                break;

            case DRIVE_BACK:
                // The starter's deadline group: spin up 0.5 s into the drive, end with the drive.
                if (!spunUpOnTheWay && stateTimer.seconds() >= SPIN_UP_AFTER_S) {
                    mech.spinUp();
                    spunUpOnTheWay = true;
                }
                if (!follower.isBusy()) next();
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
            case DRIVE_OUT:
                mech.spinUp();
                go(Paths.line(here(), p(SCORE_SOUTH)).tangent(), DEFAULT_SPEED);
                break;
            case SHOOT_PRELOADS:
            case SHOOT_FLOWER:
            case SHOOT_GARDEN:
                mech.volley(VOLLEY);
                break;

            case TO_OTHER_SIDE:
                go(curve(TO_OTHER_SIDE, FLOWER_NORTH_APPROACH).heading(pieces(TO_OTHER_SIDE_HEADING)), DEFAULT_SPEED);
                break;
            case COLLECT_FLOWER:
                mech.flowerBarDown();
                mech.collect();
                // Heading held square to the wall: a tangent heading would turn a small sideways
                // offset into a yaw that swings the mouth off the flower's opening.
                go(Paths.line(here(), p(FLOWER_NORTH_PICKUP)).constant(h(FACING_NORTH)), PICKUP_SPEED);
                break;
            case WAIT_FLOWER_FULL:
            case WAIT_GARDEN_FULL:
                mech.waitingForPollen();   // at the pickup now: the no-progress clock starts here
                break;
            case TO_SCORE_NORTH:
                mech.intakeOff();
                mech.flowerBarStow();
                go(curve(TO_SCORE_NORTH, SCORE_NORTH).heading(pieces(TO_SCORE_NORTH_HEADING)), DEFAULT_SPEED);
                break;

            case TO_GARDEN:
                go(curve(TO_GARDEN, ABOVE_GARDEN_LANE).heading(pieces(TO_GARDEN_HEADING)), DEFAULT_SPEED);
                break;
            case SLIDE_ONTO_LANE:
                // A short, slow slide down onto the lane, square to the south wall: near it the 22 in
                // robot's corners swing into the wall if it turns at all.
                go(Paths.line(here(), p(GARDEN_SWEEP_START)).constant(h(FACING_WEST)), SLIDE_SPEED);
                break;
            case SWEEP_GARDEN:
                mech.collect();
                // West along the row, intake toward the red alliance wall, heading held square to the
                // south wall: the pollen come in one after another.
                go(Paths.line(here(), p(GARDEN_SWEEP_END)).constant(h(FACING_WEST)), GARDEN_SWEEP_SPEED);
                break;
            case OFF_GARDEN_LANE:
                mech.intakeOff();
                go(Paths.line(here(), p(OFF_GARDEN_LANE)).constant(h(FACING_WEST)), OFF_LANE_SPEED);
                break;
            case DRIVE_BACK:
                spunUpOnTheWay = false;
                go(curve(DRIVE_BACK, SCORE_NORTH).heading(pieces(DRIVE_BACK_HEADING)), DEFAULT_SPEED);
                break;

            case PARK:
                mech.stopAll();
                // A quarter turn clockwise, from facing the hive to facing west, on the way.
                go(Paths.line(here(), p(PARK)).heading(Interpolator.linear(h(FACING_SOUTH), h(FACING_WEST))),
                        DEFAULT_SPEED);
                break;

            case DONE:
                mech.stopAll();
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

    /** A Bezier curve from where the robot is, through the RED control points, to the RED end. */
    private Path curve(Pose[] controls, Pose end) {
        Pose[] pts = new Pose[controls.length + 2];
        pts[0] = here();
        for (int i = 0; i < controls.length; i++) pts[i + 1] = p(controls[i]);
        pts[pts.length - 1] = p(end);
        return Paths.curve(pts);
    }

    /** A path's heading built from AutoPoses' pieces (RED headings, as seen by this alliance). */
    private Interpolator pieces(HeadingPiece[] pieces) {
        com.pedropathing.paths.interpolator.PiecewiseInterpolator out = Interpolator.piecewise();
        for (HeadingPiece piece : pieces) {
            Interpolator i;
            switch (piece.mode) {
                case "linear":
                    i = Interpolator.linear(h(piece.from), h(piece.to));
                    break;
                case "tangent":
                    i = Interpolator.tangent;
                    break;
                case "tangentReversed":
                    i = Interpolator.tangent.reverse();
                    break;
                case "constant":
                    i = Interpolator.constant(h(piece.from));
                    break;
                default:
                    throw new IllegalArgumentException("unknown heading mode " + piece.mode);
            }
            out.until(piece.endT, i);
        }
        return out;
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
