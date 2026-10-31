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
    public static double PICKUP_SPEED = 0.2;    // SET BY HAND (the team): slower into the flower
    public static double GARDEN_SWEEP_SPEED = 0.2;   // SET BY HAND (team): slower along the garden row, so every pollen goes in
    public static double SLIDE_SPEED = 0.3;     // SET BY HAND: slow sideways onto the garden lane, so it stops on time
    public static double OFF_LANE_SPEED = 0.6;  // SET BY HAND: up off the garden lane, sideways and back
    public static double SWEEP_TIMEOUT_S = 3.0;       // SET BY HAND: a stuck sweep gives up after this
    public static double FLOWER_WAIT_S = 1.5;         // SET BY HAND (the team): longer at the flower
    public static double FLOWER_DWELL_S = 0.5;        // SET BY HAND (the team): intake on, bar down, after full
    public static double GARDEN_WAIT_S = 0.8;         // SET BY HAND
    public static double SPIN_UP_AFTER_S = 0.5;       // the starter's: spin up 0.5 s into the drive back
    // Running late (a slow pickup, a stuck path): the AUTO ends at 30 s wherever the robot is. Times
    // are from PLAY. SET FROM SIM: the planned route leaves the garden at 16.7 s and parks at 25.0 s;
    // leaving the garden to park takes 8.3 s, a last volley plus parking 4.3 s; each cutoff keeps
    // ~0.5 s in hand. Parking matters only if BIOBUZZ scores it: check the game manual.
    /** Still at the garden at this time: leave it now with what is aboard. */
    public static double LEAVE_GARDEN_BY_S = 20.5;
    /** Reaching the last volley at or after this time: skip it and park. */
    public static double SKIP_LAST_VOLLEY_AFTER_S = 25.2;
    public static final int PRELOAD = 4;              // SET BY HAND: the robot starts full (G407 max)
    public static final int VOLLEY = 4;

    public enum State {
        DRIVE_OUT,              // start -> south score spot
        SHOOT_PRELOADS,         // tip 1 (3 nectar staged in the south cell)
        TO_OTHER_SIDE,          // round the west of the hive to below flower1
        DROP_FLOWER_BAR,        // bar down and intake on, standing still until the bar is down
        COLLECT_FLOWER,         // bar down, intake on, slowly in; ends at the opening or when full
        WAIT_FLOWER_FULL,
        FLOWER_DWELL,           // intake still running and the bar still down a little longer
        TO_SCORE_NORTH,         // the starter's leg: backs off the flower facing it, bar still down, turns round
        SHOOT_FLOWER,           // the volley lifts the bar to SHOOTING
        TO_GARDEN,              // bar stowed; back round the west of the hive, ending heading west above the lane
        SLIDE_ONTO_LANE,        // down onto the lane, square to the south wall
        SWEEP_GARDEN,           // intake on, west along the row (bar stowed): the pollen come in a line
        WAIT_GARDEN_FULL,
        OFF_GARDEN_LANE,        // up off the lane still facing west
        DRIVE_BACK,             // round to the north score spot, spinning up on the way
        SHOOT_GARDEN,           // tip 2
        TO_PARK_LANE,           // out toward the north wall and west along it, turning to face the field
        PARK,                   // slide south into the park spot: back to the west wall, intake to the field
        DONE
    }

    private BioHorsRobot robot;
    private Follower follower;
    private ForesightConfig foresight;
    private AutoMechanisms mech;
    private PinpointWatchdog watchdog;
    private boolean halted;
    private final java.util.List<String> lateCuts = new java.util.ArrayList<>();
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
        // The gate opens only at a scoring spot (ShootingPose), wherever the follower really is.
        // No pose, no scoring-spot check: a tripped watchdog keeps the gate shut.
        watchdog = new PinpointWatchdog(robot::pinpointReady);
        mech = new AutoMechanisms(robot, PRELOAD,
                () -> !watchdog.tripped() && ShootingPose.at(alliance, follower.pose()));
        publish();
    }

    private void initLoop() {
        telemetry.addData("Alliance", "%s (change it with the Choose Alliance OpMode)", alliance);
        telemetry.addData("Pinpoint", robot.pinpointReady() ? "READY" : "NOT READY");
        telemetry.addData("Start", "%s", p(START));
        java.util.List<String> checks = org.firstinspires.ftc.teamcode.utilities.PreMatchCheck.lines(robot);
        telemetry.addData("Check", org.firstinspires.ftc.teamcode.utilities.PreMatchCheck.allOk(checks)
                ? "ALL OK" : "FIX BEFORE PLAY");
        for (String c : checks) telemetry.addLine("  " + c);
    }

    private void startAuto() {
        matchTimer.reset();
        watchdog.start();
        enter(State.DRIVE_OUT);
    }

    private void loopOnce() {
        if (watchdog.update()) {
            // Odometry lost: stop driving for the rest of the auto, mechanisms off, gates shut.
            if (!halted) {
                halted = true;
                follower.stop();
                mech.stopAll();
            }
            // robot.stop() writes zero to the drive and intake and runs the shooter loops switched
            // off; mech.update() (which would) is skipped while halted.
            robot.stop();
            telemetry.addData("AUTO HALTED", watchdog.reason());
            telemetry.addData("State", "%s (halted, %.1f s total)", state, matchTimer.seconds());
            return;
        }
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
        if (!lateCuts.isEmpty()) telemetry.addData("LATE", "%s", lateCuts);
        if (mech.spinUpFailures() > 0) {
            telemetry.addData("SHOOTER", "never reached speed: %d volley(s) skipped, pollen kept", mech.spinUpFailures());
        }
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

    /**
     * Where a late auto goes instead, or null if it is on time. Only where the planned path onward
     * stays safe: off the garden lane (its own leg), or the park legs from the north score spot.
     */
    static State lateCut(State s, double sincePlayS, double enteredAtS) {
        switch (s) {
            case SLIDE_ONTO_LANE:
            case SWEEP_GARDEN:
            case WAIT_GARDEN_FULL:
                return sincePlayS >= LEAVE_GARDEN_BY_S ? State.OFF_GARDEN_LANE : null;
            case SHOOT_GARDEN:
                // Judged by when the volley was reached: one already under way is let finish.
                return enteredAtS >= SKIP_LAST_VOLLEY_AFTER_S ? State.TO_PARK_LANE : null;
            default:
                return null;
        }
    }

    /** Called every loop: has the current state finished? If so, move on. */
    private void run() {
        State late = lateCut(state, matchTimer.seconds(), matchTimer.seconds() - stateTimer.seconds());
        if (late != null) {
            lateCuts.add(String.format(java.util.Locale.ROOT, "%s -> %s at %.1f s", state, late, matchTimer.seconds()));
            enter(late);
            return;
        }
        switch (state) {
            case SHOOT_PRELOADS:
            case SHOOT_FLOWER:
            case SHOOT_GARDEN:
                if (mech.volleyDone()) next();
                break;

            case DROP_FLOWER_BAR:
                if (stateTimer.seconds() >= MechanismConstants.FLOWER_BAR_DROP_S) next();
                break;
            case COLLECT_FLOWER:
                // Always all the way in to the pickup (the team: closer); the wait after it covers full.
                if (!follower.isBusy()) next();
                break;
            case FLOWER_DWELL:
                if (stateTimer.seconds() >= FLOWER_DWELL_S) next();
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
            case DROP_FLOWER_BAR:
                // The team: drop the bar first, then drive in to intake.
                mech.flowerBarDown();
                mech.collect();
                break;
            case COLLECT_FLOWER:
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
                // The bar stays down all the way back (the team); the volley lifts it to SHOOTING.
                go(curve(TO_SCORE_NORTH, SCORE_NORTH).heading(pieces(TO_SCORE_NORTH_HEADING)), DEFAULT_SPEED);
                break;

            case TO_GARDEN:
                mech.flowerBarStow();   // folded after the north volley, for the rest of the route
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

            case TO_PARK_LANE:
                mech.stopAll();
                // A quarter turn counter-clockwise, from facing the hive to facing the field (east), on the way.
                go(curve(TO_PARK_LANE, PARK_LANE).heading(Interpolator.linear(h(FACING_SOUTH), h(FACING_EAST))),
                        DEFAULT_SPEED);
                break;
            case PARK:
                // Slide south (the robot's right) into the spot, still facing the field.
                go(Paths.line(here(), p(PARK)).constant(h(FACING_EAST)), DEFAULT_SPEED);
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

    /** The late cuts taken this run ("STATE -> STATE at t s"). */
    public java.util.List<String> lateCuts() {
        return lateCuts;
    }

    /** True once the Pinpoint watchdog has halted the auto. */
    public boolean halted() {
        return halted;
    }
}
