package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.aim.AimController;
import org.firstinspires.ftc.teamcode.aim.AimVelocityFilter;
import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.firstinspires.ftc.teamcode.aim.DistanceSpeedTable;
import org.firstinspires.ftc.teamcode.aim.FieldTargets;
import org.firstinspires.ftc.teamcode.aim.FireGate;
import org.firstinspires.ftc.teamcode.aim.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.IntakeController;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;

/**
 * The MainHORS teleop: the MainHORS control scheme on this robot, plus odometry aim.
 *
 * Controls (gamepad1; gamepad2 mirrors the mechanism buttons):
 *   Left stick ............ drive (forward / strafe)
 *   Right stick X ......... turn (overridden while aiming)
 *   Left trigger (hold) ... AIM: turn the robot to face our hive's cell on this half of the field,
 *                           and spin both shooters up (see "Spin-up while aiming" below)
 *   Right bumper (hold) ... slow mode (0.45x)
 *   Right trigger ......... intake in (a stalled intake reverses briefly on its own: IntakeJamGuard)
 *   Y (hold) .............. outtake
 *   Dpad down ............. toggle both shooters on/off (each at its own target), for spinning
 *                           without aiming
 *   Dpad left / right ..... POLLEN shooter target RPM - / + (MechanismConstants.POLLEN_TRIM_STEP_RPM)
 *   Left bumper / A ....... NECTAR shooter target RPM - / + (MechanismConstants.NECTAR_TRIM_STEP_RPM)
 *   B ..................... request / cancel both gates open (the flower bar lifts slightly while open)
 *   X ..................... RESET the pose: back the robot flat against the alliance's wall at the
 *                           loading zone's edge (the auto's park spot), intake facing the field, and
 *                           press X: the pose becomes RESET_X/Y/HEADING (mirrored for BLUE), wiping
 *                           the Pinpoint's drift.
 *   Dpad up ............... toggle the flower bar: DOWN (collect from a flower) / STOWED

 *
 * Before PLAY: dpad left = RED, dpad right = BLUE. If an auto ran first, its alliance and last
 * pose are picked up from the blackboard.
 *
 * Aim uses the Pinpoint pose and the hive's fixed field coordinates; there is no camera. The
 * target cell follows the robot's half of the field (north cell when y > 0, south when y < 0),
 * because a launch only scores from the up cell's own quarter. If the
 * Pinpoint is not READY or the pose is not a number, aiming falls back to manual turn and
 * telemetry warns. Drive and strafe stay with the driver while aiming.
 *
 * Shoot while moving:
 *   - Each shooter's target rpm is its distance-to-rpm table (MechanismConstants; one flat point at
 *     the fixed target until measured) plus that shooter's trim. Pose invalid: the fixed target
 *     plus trim.
 *   - The aim leads the shot: it turns to a point shifted against the robot's velocity by the
 *     ball's flight time (AimController.leadAimPoint). The ball speed is unset, so lead is zero.
 *   - The aim's velocity is the Pinpoint's own, smoothed (AimVelocityFilter); the pose change per
 *     loop is the backup. No usable velocity: zero, so aim runs on its P term alone.
 *   - B only REQUESTS the gates open. They open while the request stands and the fire interlock
 *     (aim/FireGate) is clear: pose valid, on target, every switched-on shooter at speed. They
 *     close on their own when it is not, and telemetry says why. B again cancels the request.
 *
 * Spin-up while aiming:
 *   - Holding aim with a usable pose switches both shooters on, without dpad down. Letting go
 *     switches them off again, unless dpad down has them on.
 *   - A volley is not cut short: while B's gate request stands, the aim's spin-up holds, even with
 *     the trigger released, until B cancels. Holding it only while the gates were OPEN let a speed
 *     dip shut them, end the spin-up, and then reopen them into stopped flywheels (the interlock
 *     ignores a shooter that is off).
 *   - With the pose invalid, aiming does not spin up: the interlock would keep the gates shut anyway.
 *
 * Intake jam: while right trigger pulls in, an intake that stalls (both encoders far below a full
 * robot's speed) runs outward for a moment, then pulls in again. The Fire line says INTAKE JAM.
 *
 * Shooters start OFF and gates CLOSED. Tuning values are placeholders (MechanismConstants,
 * AimController).
 */
@TeleOp(name = "MainHORS TeleOp", group = "Linear OpMode")
public class MainHorsTeleOp extends LinearOpMode {

    private static final double NORMAL_DRIVE_SCALE = 1.0;
    private static final double SLOW_DRIVE_SCALE = 0.45;
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double RUMBLE_COOLDOWN_S = 1.2;

    // Where the robot is assumed to start when no auto ran before this teleop. RED values; BLUE
    // is the point mirror. The auto's `park` waypoint, where AUTO ends: by the west wall next to
    // the loading zone, the intake facing the field. Not measured on a real field.
    public static double DEFAULT_START_X_IN = -61.5;   // SET BY HAND (= waypoints.json park): the back 1.37 in off the CAD west wall
    public static double DEFAULT_START_Y_IN = 49.0;    // SET BY HAND (the team, 2026-10-30; = waypoints.json park). NEEDS MEASUREMENT
    public static double DEFAULT_START_HEADING_RAD = 0.0; // the intake facing the field (east), back to the west wall

    // Where X resets the pose to (RED; BLUE is the point mirror): the robot backed flat against
    // the west wall at the park spot, the intake facing the field. The wall fixes x and the
    // heading exactly; y is the park's, by eye against the loading zone's edge.
    public static double RESET_X_IN = -62.87;          // MEASURED FROM CAD: FIRST's CAD west wall -70.674 + the back's 7.8
    public static double RESET_Y_IN = 49.0;            // SET BY HAND (= waypoints.json park). NEEDS MEASUREMENT
    public static double RESET_HEADING_RAD = 0.0;      // square to the west wall, facing the field

    // Smoothing of the aim velocity (AimVelocityFilter): 1 = raw, smaller = smoother but laggier.
    // PLACEHOLDER: NEEDS MEASUREMENT (raise it if the aim lags a strafe, lower it if it twitches).
    public static double AIM_VELOCITY_ALPHA = 0.5;
    // The pose change gives a velocity only across a loop this long, seconds (the existing guard).
    private static final double MIN_VELOCITY_DT_S = 1e-4;
    private static final double MAX_VELOCITY_DT_S = 0.25;
    // The Loop line's average covers this many loops (about 1 s at the 20 ms sim tick). SET BY HAND.
    private static final int LOOP_TIME_WINDOW = 50;

    private BioHorsRobot robot;
    private Localizer localizer;
    private final AimController aim = new AimController();

    private Alliance alliance = Alliance.RED;
    private FieldTargets.Cell targetCell = FieldTargets.upCellAtKickoff(Alliance.RED);
    private boolean poseFromAuto = false;

    private DistanceSpeedTable pollenSpeedTable;
    private DistanceSpeedTable nectarSpeedTable;
    private double pollenTrimRpm = 0.0;
    private double nectarTrimRpm = 0.0;
    private boolean gatesRequested = false;
    // Shooters on from dpad down (stays until pressed again), and on from aiming (see the class doc).
    private boolean shootersToggledOn = false;
    private boolean aimSpinUp = false;

    // Edge-detect state (a press fires once, not every loop).
    private boolean dpadDownLast, dpadLeftLast, dpadRightLast, dpadUpLast, bLast, xLast, aLast, lbLast;

    // Loop time for the driver display: the period from the start of one loop to the start of the
    // next, ms. Display only; nothing reads it to decide anything.
    private final LoopTimes loopTimes = new LoopTimes(LOOP_TIME_WINDOW);

    @Override
    public void runOpMode() {
        robot = new BioHorsRobot(hardwareMap, telemetry);
        localizer = Constants.createLocalizer(hardwareMap);
        pollenSpeedTable = new DistanceSpeedTable(
                MechanismConstants.POLLEN_SPEED_TABLE_DISTANCE_IN, MechanismConstants.POLLEN_SPEED_TABLE_RPM);
        nectarSpeedTable = new DistanceSpeedTable(
                MechanismConstants.NECTAR_SPEED_TABLE_DISTANCE_IN, MechanismConstants.NECTAR_SPEED_TABLE_RPM);
        AimVelocityFilter velocityFilter = new AimVelocityFilter(AIM_VELOCITY_ALPHA,
                Math.max(Constants.MAX_FORWARD_IN_PER_S, Constants.MAX_STRAFE_IN_PER_S));

        Alliance saved = PoseHandoff.savedAlliance();
        double[] savedPose = PoseHandoff.savedPose();
        if (saved != null) alliance = saved;
        poseFromAuto = savedPose != null;
        if (poseFromAuto) {
            localizer.setPose(new Pose(savedPose[0], savedPose[1], savedPose[2]));
        } else {
            localizer.setPose(defaultStartPose(alliance));
        }
        targetCell = FieldTargets.upCellAtKickoff(alliance);

        // ----- INIT: pick the alliance -----
        while (!isStarted() && !isStopRequested()) {
            Alliance picked = null;
            if (gamepad1.dpad_left) picked = Alliance.RED;
            if (gamepad1.dpad_right) picked = Alliance.BLUE;
            if (picked != null && picked != alliance) {
                alliance = picked;
                targetCell = FieldTargets.upCellAtKickoff(alliance);
                if (!poseFromAuto) localizer.setPose(defaultStartPose(alliance));
            }
            // Track the mechanism buttons during INIT too, so a button still held at PLAY (the
            // dpad from picking the alliance, say) is not read as a fresh press on the first loop.
            seedEdgeDetectors();
            localizer.update();
            targetCell = FieldTargets.cellForHalf(localizer.pose().y(), targetCell);
            telemetry.addData("Alliance", "%s  (dpad left = RED, dpad right = BLUE)", alliance);
            telemetry.addData("Start pose", poseFromAuto ? "from auto" : "default (unmeasured)");
            addPoseTelemetry();
            telemetry.addData("Status", "Initialized -- shooters OFF, gates CLOSED");
            telemetry.update();
        }
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        ElapsedTime loopTimer = new ElapsedTime();
        Pose lastPose = localizer.pose();
        ElapsedTime sinceRumble = new ElapsedTime();
        sinceRumble.reset();
        boolean rumbledOnce = false;
        // Its own timer, apart from loopTimer (whose dt feeds the velocity), so the readout cannot
        // disturb that math. ElapsedTime runs on sim time under SimLoop, never the wall clock.
        ElapsedTime loopTimeTimer = new ElapsedTime();
        boolean firstLoop = true;

        while (opModeIsActive()) {
            // The first loop has no previous start to measure from; skipping it also keeps the
            // INIT-to-PLAY gap out of the peak.
            if (!firstLoop) loopTimes.record(loopTimeTimer.milliseconds());
            loopTimeTimer.reset();
            firstLoop = false;

            localizer.update();
            Pose pose = localizer.pose();
            boolean poseOk = poseUsable(pose);

            // ----- Field velocity for the aim feedforward and lead: the Pinpoint's own, else the
            // pose change over the loop; smoothed and clamped. None usable: zero this loop. -----
            double dt = loopTimer.seconds();
            loopTimer.reset();
            double rawVx = Double.NaN;
            double rawVy = Double.NaN;
            if (poseOk) {
                Velocity pv = pinpointVelocity();
                if (pv != null && Double.isFinite(pv.vx) && Double.isFinite(pv.vy)) {
                    rawVx = pv.vx;
                    rawVy = pv.vy;
                } else if (lastPose != null && dt > MIN_VELOCITY_DT_S && dt < MAX_VELOCITY_DT_S
                        && Double.isFinite(lastPose.x()) && Double.isFinite(lastPose.y())) {
                    rawVx = (pose.x() - lastPose.x()) / dt;
                    rawVy = (pose.y() - lastPose.y()) / dt;
                }
            }
            double[] v = velocityFilter.update(rawVx, rawVy);
            double vx = v[0];
            double vy = v[1];
            lastPose = pose;

            // ----- Target: the cell of our hive on the robot's half of the field -----
            if (poseOk) targetCell = FieldTargets.cellForHalf(pose.y(), targetCell);
            double[] cell = FieldTargets.cellAimPoint(alliance, targetCell);
            // The flywheel speed uses the TRUE distance; the aim uses the lead-shifted point.
            double distanceIn = poseOk ? Math.hypot(cell[0] - pose.x(), cell[1] - pose.y()) : Double.NaN;
            double[] aimPoint = poseOk
                    ? AimController.leadAimPoint(pose.x(), pose.y(), vx, vy, cell[0], cell[1],
                            AimController.BALL_GROUND_SPEED_IN_PER_S)
                    : cell;
            double headingErrorRad = poseOk
                    ? AimController.wrap(AimController.desiredHeading(pose.x(), pose.y(), aimPoint[0], aimPoint[1])
                            - pose.heading())
                    : Double.NaN;

            // ----- Drive, with aim overriding manual turn while left trigger is held -----
            double scale = gamepad1.right_bumper ? SLOW_DRIVE_SCALE : NORMAL_DRIVE_SCALE;
            double axial = DriveController.shapeTranslation(-gamepad1.left_stick_y);
            double lateral = DriveController.shapeTranslation(gamepad1.left_stick_x);
            boolean aimHeld = gamepad1.left_trigger > TRIGGER_THRESHOLD;
            boolean aiming = aimHeld && poseOk;
            if (aiming) {
                double yaw = aim.yawCommand(pose.x(), pose.y(), pose.heading(), vx, vy,
                        aimPoint[0], aimPoint[1]);
                // Yaw has priority: translation gets only the headroom the aim yaw leaves, so the
                // mecanum normalization never shrinks the yaw while the driver strafes hard. The
                // aim yaw is not slowed by slow mode: it is already capped.
                double a = axial * scale;
                double l = lateral * scale;
                double room = 1.0 - Math.abs(yaw);
                double used = Math.abs(a) + Math.abs(l);
                if (used > room && used > 1e-9) {
                    a *= room / used;
                    l *= room / used;
                }
                robot.drive.setDriveShaped(a, l, yaw, 1.0);
            } else {
                robot.drive.setDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                        gamepad1.right_stick_x, scale);
            }

            // ----- Shooters: on while dpad down has them toggled on, or while aiming (held through
            // an open volley). Dpad left/right trims pollen. Each target is its distance table's rpm
            // plus its trim. -----
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) shootersToggledOn = !shootersToggledOn;
            dpadDownLast = dpadDownNow;
            aimSpinUp = aiming || (aimSpinUp && gatesRequested);
            boolean shootersOn = shootersToggledOn || aimSpinUp;
            robot.pollenShooter.setShooterOn(shootersOn);
            robot.nectarShooter.setShooterOn(shootersOn);

            double pollenTableRpm = pollenSpeedTable.rpmAt(distanceIn, MechanismConstants.POLLEN_SHOOTER.targetRpm);
            double nectarTableRpm = nectarSpeedTable.rpmAt(distanceIn, MechanismConstants.NECTAR_SHOOTER.targetRpm);
            double pollenTrimBefore = pollenTrimRpm;
            double nectarTrimBefore = nectarTrimRpm;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) pollenTrimRpm = trimmed(pollenTrimRpm, -MechanismConstants.POLLEN_TRIM_STEP_RPM, pollenTableRpm);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) pollenTrimRpm = trimmed(pollenTrimRpm, MechanismConstants.POLLEN_TRIM_STEP_RPM, pollenTableRpm);
            dpadRightLast = dpadRightNow;

            // Nectar has its own trim and step: left bumper down, A up.
            boolean lbNow = gamepad1.left_bumper || gamepad2.left_bumper;
            if (lbNow && !lbLast) nectarTrimRpm = trimmed(nectarTrimRpm, -MechanismConstants.NECTAR_TRIM_STEP_RPM, nectarTableRpm);
            lbLast = lbNow;

            boolean aNow = gamepad1.a || gamepad2.a;
            if (aNow && !aLast) nectarTrimRpm = trimmed(nectarTrimRpm, MechanismConstants.NECTAR_TRIM_STEP_RPM, nectarTableRpm);
            aLast = aNow;

            applyTarget(robot.pollenShooter, pollenTableRpm, pollenTrimRpm, pollenTrimRpm != pollenTrimBefore);
            applyTarget(robot.nectarShooter, nectarTableRpm, nectarTrimRpm, nectarTrimRpm != nectarTrimBefore);

            robot.updateMechanisms();

            // Rumble once (with cooldown) when either shooter first reaches its target.
            // Read both flags every loop so neither stays latched.
            boolean pollenReached = robot.pollenShooter.justReachedTarget();
            boolean nectarReached = robot.nectarShooter.justReachedTarget();
            if ((pollenReached || nectarReached)
                    && (!rumbledOnce || sinceRumble.seconds() >= RUMBLE_COOLDOWN_S)) {
                try {
                    gamepad1.rumble(220);
                    gamepad2.rumble(220);
                } catch (Throwable ignored) {
                }
                sinceRumble.reset();
                rumbledOnce = true;
            }

            // ----- Gates: B requests / cancels both; they open only while the interlock is clear -----
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bLast) gatesRequested = !gatesRequested;
            bLast = bNow;
            FireGate.Decision fire = FireGate.decide(poseOk, headingErrorRad, AimController.ON_TARGET_TOLERANCE_RAD,
                    fireState(robot.pollenShooter), fireState(robot.nectarShooter));
            boolean gatesOpen = gatesRequested && fire.clear;
            robot.pollenGate.setClosed(!gatesOpen);
            robot.nectarGate.setClosed(!gatesOpen);

            // ----- Flower bar: dpad up toggles down / stowed; lifted slightly while the gates are open -----
            boolean dpadUpNow = gamepad1.dpad_up || gamepad2.dpad_up;
            if (dpadUpNow && !dpadUpLast) robot.flowerBar.toggle();
            dpadUpLast = dpadUpNow;

            // ----- X: reset the pose to the reset spot (wipes Pinpoint drift). The driver puts the
            // robot there first: back to the west wall by the loading zone, intake facing the field. -----
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xLast) {
                localizer.setPose(resetPose(alliance));
                lastPose = null;            // no velocity across the jump
                velocityFilter.reset();
                gamepad1.rumble(120);
            }
            xLast = xNow;
            robot.flowerBar.setShooting(!robot.pollenGate.isClosed() || !robot.nectarGate.isClosed());

            // ----- Intake: right trigger in, Y out; a stalled intake reverses briefly -----
            double intakeRequest = 0.0;
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD || gamepad2.right_trigger > TRIGGER_THRESHOLD) {
                intakeRequest = IntakeController.INTAKE_POWER;
            } else if (gamepad1.y || gamepad2.y) {
                intakeRequest = IntakeController.OUTTAKE_POWER;
            }
            robot.intake.setPower(robot.intakeJam.power(intakeRequest));

            // ----- Telemetry: the driving display, five lines and nothing else -----
            String fireText = fire.clear ? "READY" : "BLOCKED: " + fire.blockedReason;
            if (robot.intakeJam.reversing()) fireText += " | INTAKE JAM: reversing";
            telemetry.addData("Fire", fireText);
            addPoseTelemetry();
            addShooterTelemetry("Pollen", robot.pollenShooter);
            addShooterTelemetry("Nectar", robot.nectarShooter);
            addLoopTimeTelemetry();
            telemetry.update();
        }

        robot.stop();
    }

    /** Sets every edge detector to the buttons' current state. */
    private void seedEdgeDetectors() {
        dpadDownLast = gamepad1.dpad_down || gamepad2.dpad_down;
        dpadLeftLast = gamepad1.dpad_left || gamepad2.dpad_left;
        dpadRightLast = gamepad1.dpad_right || gamepad2.dpad_right;
        xLast = gamepad1.x || gamepad2.x;
        dpadUpLast = gamepad1.dpad_up || gamepad2.dpad_up;
        aLast = gamepad1.a || gamepad2.a;
        bLast = gamepad1.b || gamepad2.b;
        lbLast = gamepad1.left_bumper || gamepad2.left_bumper;
    }

    /** The pose X resets to for an alliance: the RED value, point-mirrored for BLUE. */
    static Pose resetPose(Alliance alliance) {
        if (alliance == Alliance.RED) return new Pose(RESET_X_IN, RESET_Y_IN, RESET_HEADING_RAD);
        return new Pose(-RESET_X_IN, -RESET_Y_IN, AimController.wrap(RESET_HEADING_RAD + Math.PI));
    }

    /** The default start pose for an alliance: the RED value, point-mirrored for BLUE. */
    static Pose defaultStartPose(Alliance alliance) {
        if (alliance == Alliance.RED) {
            return new Pose(DEFAULT_START_X_IN, DEFAULT_START_Y_IN, DEFAULT_START_HEADING_RAD);
        }
        return new Pose(-DEFAULT_START_X_IN, -DEFAULT_START_Y_IN,
                AimController.wrap(DEFAULT_START_HEADING_RAD + Math.PI));
    }

    /** A pose may be used for aiming only when the Pinpoint is READY and the numbers are real. */
    private boolean poseUsable(Pose pose) {
        return robot.pinpointReady() && pose != null
                && Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }

    /** The Pinpoint's own field velocity (in/s) through the localizer, or null if it has none. */
    private Velocity pinpointVelocity() {
        try {
            return localizer.velocity();
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * A trim after one press, never taking the shooter's target below zero. The step starts from
     * the trim as it currently applies (a trim below -table only clamps the target at zero), so a
     * press always moves the target by one step even after the table rpm has dropped.
     */
    static double trimmed(double trimRpm, double stepRpm, double tableRpm) {
        double effective = Math.max(-tableRpm, trimRpm);
        return Math.max(-tableRpm, effective + stepRpm);
    }

    /**
     * Sets a shooter's target to its table rpm plus trim. A table drift smaller than
     * MechanismConstants.SPEED_TABLE_UPDATE_STEP_RPM is ignored, so pose jitter does not reset the
     * shooter's integral every loop; a trim press always applies.
     */
    private static void applyTarget(ShooterController shooter, double tableRpm, double trimRpm, boolean trimChanged) {
        double want = Math.max(0.0, tableRpm + trimRpm);
        if (!Double.isFinite(want)) return;
        if (trimChanged || Math.abs(want - shooter.getTargetRpm()) >= MechanismConstants.SPEED_TABLE_UPDATE_STEP_RPM) {
            shooter.setTargetRpm(want);
        }
    }

    private static FireGate.Shooter fireState(ShooterController s) {
        return new FireGate.Shooter(s.getLabel(), s.isShooterOn(), s.isAtTarget(), s.getTargetRpm(), s.getCurrentRpm());
    }

    private void addPoseTelemetry() {
        Pose p = localizer.pose();
        boolean ok = poseUsable(p);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f deg %s",
                p.x(), p.y(), Math.toDegrees(p.heading()), ok ? "" : "(INVALID)");
    }

    /** {@code current / target rpm STATE}: OFF, SPINNING UP (on, not at target) or OK. */
    private void addShooterTelemetry(String label, ShooterController s) {
        String state = !s.isShooterOn() ? "OFF" : s.isAtTarget() ? "OK" : "SPINNING UP";
        telemetry.addData(label, "%.0f / %.0f rpm %s", s.getCurrentRpm(), s.getTargetRpm(), state);
    }

    /** {@code last ms (avg over the last LOOP_TIME_WINDOW loops, max since PLAY)}. */
    private void addLoopTimeTelemetry() {
        if (!loopTimes.hasSample()) {
            telemetry.addData("Loop", "-- ms (avg --, max --)");
            return;
        }
        telemetry.addData("Loop", "%.1f ms (avg %.1f, max %.1f)",
                loopTimes.lastMs(), loopTimes.averageMs(), loopTimes.peakMs());
    }

    /** Loop periods, ms: the last one, the mean of the last {@code window}, and the peak of all. */
    static final class LoopTimes {
        private final double[] windowMs;
        private int count;
        private int next;
        private double lastMs = Double.NaN;
        private double peakMs = Double.NaN;

        LoopTimes(int window) {
            windowMs = new double[window];
        }

        void record(double ms) {
            lastMs = ms;
            peakMs = Double.isNaN(peakMs) ? ms : Math.max(peakMs, ms);
            windowMs[next] = ms;
            next = (next + 1) % windowMs.length;
            if (count < windowMs.length) count++;
        }

        boolean hasSample() {
            return count > 0;
        }

        double lastMs() {
            return lastMs;
        }

        /** Mean of the last {@code window} periods (fewer until that many are recorded); NaN if none. */
        double averageMs() {
            if (count == 0) return Double.NaN;
            double sum = 0.0;
            for (int i = 0; i < count; i++) sum += windowMs[i];
            return sum / count;
        }

        double peakMs() {
            return peakMs;
        }
    }
}
