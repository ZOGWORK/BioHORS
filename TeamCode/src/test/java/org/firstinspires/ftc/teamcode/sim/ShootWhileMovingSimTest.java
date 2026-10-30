package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.aim.AimController;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.opmode.GamepadScript;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.simclock.SimClockLoader;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Shoot-while-moving in the MainHORS teleop, run unchanged in SimLoop: the fire interlock on the
 * gates, the ball-flight lead while strafing, and the distance-to-rpm table.
 *
 * Assertions are on the simulated world (the plant's true pose, the gate servos' real position, the
 * flywheels' real speed) and on the Driver Station telemetry, which is how a driver sees a refusal.
 * Expected headings are worked out here from the hive coordinates and the plant's own motion, with
 * an independent closed-form lead solve, so a mistake in AimController cannot also be in the
 * expectation.
 */
class ShootWhileMovingSimTest {

    static final String TELEOP = MainHorsTeleOpSimTest.TELEOP;
    static final double TICK_S = MainHorsTeleOpSimTest.TICK_S;
    static final int INIT_LOOP_TICKS = MainHorsTeleOpSimTest.INIT_LOOP_TICKS;
    static final double RED_HIVE_X = MainHorsTeleOpSimTest.RED_HIVE_X;
    static final double CELL_Y = MainHorsTeleOpSimTest.CELL_Y;
    static final double GATE_OPEN = 0.67;
    static final double GATE_CLOSED = 0.485;
    /** The aim's on-target tolerance (AimController.ON_TARGET_TOLERANCE_RAD), repeated here. */
    static final double TOL_DEG = 2.0;

    @AfterEach
    void clearBlackboard() {
        OpMode.blackboard.clear();
    }

    // ------------------------------------------------------------------ fire interlock

    @Test
    void gatesRefuseToOpenWhileAFlywheelIsSlowAndTelemetrySaysSo() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, -13.0, -63.0, Math.PI / 2.0); // faces the RED south cell already
        final int[] closedWhileSlow = {0};
        final String[] openedEarly = {null};
        OpModeRunResult result = run(sim, TELEOP, 250, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5; // shooters on, from rest
            g1.b = tick >= 2 && tick < 5;         // request the gates at the same moment
        }, (tick, s) -> {
            // The flywheels' TRUE speed, against the 75 rpm tolerance plus a little filter lag.
            boolean slow = Math.abs(s.pollenShooterRpm() - 1600.0) > 150.0
                    || Math.abs(s.nectarShooterRpm() - 1600.0) > 150.0;
            boolean moving = s.pollenGatePlant.getPosition() > GATE_CLOSED + 0.02
                    || s.nectarGatePlant.getPosition() > GATE_CLOSED + 0.02;
            if (tick > 2 && slow && !moving) closedWhileSlow[0]++;
            if (slow && moving && openedEarly[0] == null) {
                openedEarly[0] = String.format("tick %d: pollen %.0f rpm, nectar %.0f rpm",
                        tick, s.pollenShooterRpm(), s.nectarShooterRpm());
            }
        });

        assertEquals(null, openedEarly[0], "a gate opened while a flywheel was slow");
        assertTrue(closedWhileSlow[0] >= 3, "the request should have waited on the spin-up for a few ticks, "
                + closedWhileSlow[0]);
        assertTrue(anyLine(result, "Fire", "BLOCKED: Pollen flywheel"),
                "telemetry should name the slow flywheel");
        assertFalse(anyLine(result, "Fire", "AIM"), "the aim was on target the whole time");
        assertEquals(GATE_OPEN, sim.pollenGatePlant.getPosition(), 0.02,
                "once both flywheels are at speed the standing request opens the gate");
        assertEquals(GATE_OPEN, sim.nectarGatePlant.getPosition(), 0.02);
        assertTrue(lastLine(result, "Fire").contains("READY"), lastLine(result, "Fire"));
    }

    @Test
    void gatesRefuseToOpenOffTargetAndOpenOnceTheAimArrives() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, -40.0, -40.0, 0.0); // facing +X, about 45 deg off the south cell
        double[] gateAt = new double[260];
        OpModeRunResult result = run(sim, TELEOP, 200, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.b = tick >= 2 && tick < 5;              // shooters off: only the aim can block
            g1.left_trigger = tick >= 60 ? 1f : 0f;    // then aim
        }, (tick, s) -> gateAt[tick] = s.pollenGatePlant.getPosition());

        assertEquals(GATE_CLOSED, gateAt[50], 0.02, "off target: the gate must stay closed");
        assertTrue(anyLine(result, "Fire", "BLOCKED: AIM"), "telemetry should name the aim error");
        double want = Math.atan2(-CELL_Y - sim.drivePlant.getY(), RED_HIVE_X - sim.drivePlant.getX());
        assertTrue(Math.toDegrees(Math.abs(wrap(sim.drivePlant.getHeading() - want))) <= TOL_DEG + 0.5,
                "the aim should have arrived");
        assertEquals(GATE_OPEN, sim.pollenGatePlant.getPosition(), 0.02, "on target: the gate opens");
    }

    @Test
    void anOpenGateClosesOnItsOwnWhenTheAimIsLost() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, -13.0, -63.0, Math.PI / 2.0);
        double[] gateAt = new double[160];
        OpModeRunResult result = run(sim, TELEOP, 120, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.b = tick >= 2 && tick < 5;
            g1.right_stick_x = tick >= 30 && tick < 60 ? 0.5f : 0f; // the driver turns away
        }, (tick, s) -> gateAt[tick] = s.pollenGatePlant.getPosition());

        assertEquals(GATE_OPEN, gateAt[25], 0.02, "on target with the shooters off: open");
        assertEquals(GATE_CLOSED, sim.pollenGatePlant.getPosition(), 0.02, "turned away: closed on its own");
        assertTrue(lastLine(result, "Fire").contains("BLOCKED: AIM"), lastLine(result, "Fire"));
        assertTrue(lastLine(result, "Gates").contains("requested=yes"), "the request still stands");
    }

    @Test
    void anInvalidPoseKeepsTheGatesClosedAndSaysSo() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, -13.0, -63.0, Math.PI / 2.0);
        final int[] n = {0};
        OpModeRunResult result = OpModeRunner.forTeamClass(TELEOP, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(60)
                .gamepads((phase, tick, g1, g2) -> {
                    if (phase != OpModePhase.ACTIVE) return;
                    if (n[0]++ == 0) sim.pinpoint.forceFault(GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED);
                    g1.b = tick >= 2 && tick < 5;
                })
                .eachTick(sim::step)
                .run();
        assertEquals(GATE_CLOSED, sim.pollenGatePlant.getPosition(), 0.02);
        assertTrue(lastLine(result, "Fire").contains("POSE INVALID"), lastLine(result, "Fire"));
        assertTrue(lastLine(result, "Pollen shooter").contains("target=1600"),
                "pose invalid: the fixed rpm, " + lastLine(result, "Pollen shooter"));
    }

    // ------------------------------------------------------------------ aim while strafing

    @Test
    void strafingWithAimHeldStaysOnTheCellWithLeadOff() {
        StrafeRun r = strafe(TELEOP, null, Paths.get("build", "sim", "teleop-strafe-aim.rlog"));
        assertTrue(r.moved > 15.0, "the robot should strafe while aiming, moved " + r.moved);
        assertTrue(r.worstErrToCellDeg <= 3.0, String.format(
                "lead off: aim should stay on the cell while strafing; worst %.2f deg", r.worstErrToCellDeg));
        assertTrue(lastLine(r.result, "Lead").contains("DISABLED"), "telemetry says lead is disabled");
    }

    @Test
    void withABallSpeedSetTheAimHoldsTheLeadShiftedHeadingWhileStrafing() {
        StrafeRun r = strafe(ConfiguredTeleOp.CLASS, LEAD_BALL_SPEED, Paths.get("build", "sim", "teleop-strafe-lead.rlog"));
        assertTrue(r.moved > 15.0, "moved " + r.moved);
        assertTrue(r.worstErrToLeadDeg <= 3.0, String.format(
                "lead on: aim should hold the lead heading; worst %.2f deg", r.worstErrToLeadDeg));
        assertTrue(r.meanLeadDeg >= 3.0, String.format("the lead should be visible: mean %.2f deg", r.meanLeadDeg));
        assertTrue(r.meanErrToCellDeg >= 2.0, String.format(
                "and the robot should NOT point straight at the cell: mean %.2f deg", r.meanErrToCellDeg));
    }

    /** Ball ground speed for the lead runs, in/s. A test value, not a measurement. */
    static final double LEAD_BALL_SPEED = 150.0;

    static final class StrafeRun {
        OpModeRunResult result;
        double moved;
        double worstErrToCellDeg;
        double worstErrToLeadDeg;
        double meanLeadDeg;
        double meanErrToCellDeg;
    }

    /**
     * Locks on from (-40, -40) for 1.5 s, then strafes (robot-frame left at 0.4 stick) for 1.5 s with
     * the trigger held. Errors are measured on the true pose from 0.5 s into the strafe.
     */
    private static StrafeRun strafe(String opMode, Double ballSpeed, Path rlog) {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, -40.0, -40.0, 0.0);
        if (ballSpeed != null) OpMode.blackboard.put(ConfiguredTeleOp.BALL_SPEED, ballSpeed);
        StrafeRun r = new StrafeRun();
        final OpModePhase[] phaseNow = {OpModePhase.INIT};
        final int[] tickNow = {0};
        final double[] last = {Double.NaN, Double.NaN};
        final double[] start = new double[2];
        final double[] sums = new double[3]; // lead deg, error to cell deg, samples
        r.result = OpModeRunner.forTeamClass(opMode, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(150)
                .gamepads((phase, tick, g1, g2) -> {
                    phaseNow[0] = phase;
                    tickNow[0] = tick;
                    if (phase != OpModePhase.ACTIVE) return;
                    g1.left_trigger = 1f;
                    g1.left_stick_x = tick >= 75 ? -0.4f : 0f;
                })
                .eachTick(dt -> {
                    sim.step(dt);
                    MainHorsTeleOpSimTest.logTruth(sim);
                    double x = sim.drivePlant.getX(), y = sim.drivePlant.getY(), h = sim.drivePlant.getHeading();
                    double vx = (x - last[0]) / dt, vy = (y - last[1]) / dt;
                    last[0] = x;
                    last[1] = y;
                    if (phaseNow[0] != OpModePhase.ACTIVE) return;
                    if (tickNow[0] == 75) {
                        start[0] = x;
                        start[1] = y;
                    }
                    double cellY = -CELL_Y; // the strafe stays south of the centre line (asserted below)
                    double toCell = Math.atan2(cellY - y, RED_HIVE_X - x);
                    double toLead = ballSpeed == null ? toCell : leadHeading(x, y, vx, vy, RED_HIVE_X, cellY, ballSpeed);
                    Logger.recordOutput("Aim/expectedHeadingRad", toLead);
                    if (tickNow[0] < 100) return;
                    assertTrue(y < -2.0, "the strafe must stay in the south half, y = " + y);
                    double errCell = Math.toDegrees(Math.abs(wrap(h - toCell)));
                    double errLead = Math.toDegrees(Math.abs(wrap(h - toLead)));
                    r.worstErrToCellDeg = Math.max(r.worstErrToCellDeg, errCell);
                    r.worstErrToLeadDeg = Math.max(r.worstErrToLeadDeg, errLead);
                    sums[0] += Math.toDegrees(Math.abs(wrap(toLead - toCell)));
                    sums[1] += errCell;
                    sums[2] += 1;
                })
                .runLogged(BioHorsSim.ROBOT_CONFIG, rlog);
        r.moved = Math.hypot(sim.drivePlant.getX() - start[0], sim.drivePlant.getY() - start[1]);
        r.meanLeadDeg = sums[0] / sums[2];
        r.meanErrToCellDeg = sums[1] / sums[2];
        System.out.printf(java.util.Locale.ROOT,
                "STRAFE ball=%s moved=%.1f in  worst to cell %.2f deg  worst to lead %.2f deg  mean lead %.2f deg%n",
                ballSpeed, r.moved, r.worstErrToCellDeg, r.worstErrToLeadDeg, r.meanLeadDeg);
        return r;
    }

    /**
     * Heading that leads a target for a ball of ground speed s fired from a robot moving at v: the
     * aim point T - v t with |T - v t - R| = s t, solved in closed form (a quadratic in t), not by
     * the robot code's iteration. Limited to 10 deg either side of the true bearing, as the spec says.
     */
    static double leadHeading(double rx, double ry, double vx, double vy, double tx, double ty, double s) {
        double dx = tx - rx, dy = ty - ry;
        double a = vx * vx + vy * vy - s * s;
        double b = -2.0 * (dx * vx + dy * vy);
        double c = dx * dx + dy * dy;
        double t = (-b - Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a); // a < 0: the positive root
        double bearing = Math.atan2(dy, dx);
        double lead = wrap(Math.atan2(ty - vy * t - ry, tx - vx * t - rx) - bearing);
        return bearing + Math.max(-Math.toRadians(10.0), Math.min(Math.toRadians(10.0), lead));
    }

    // ------------------------------------------------------------------ flywheel speed by distance

    @Test
    void theFlywheelTargetFollowsTheDistanceTableAndTheTrimAddsOnTop() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, -13.0, -63.0, Math.PI / 2.0);
        OpMode.blackboard.put(ConfiguredTeleOp.POLLEN_TABLE_DISTANCE, new double[] {30.0, 70.0});
        OpMode.blackboard.put(ConfiguredTeleOp.POLLEN_TABLE_RPM, new double[] {1500.0, 2300.0});
        double[] targetAtStart = {Double.NaN};
        final int[] lines = {0};
        OpModeRunResult result = run(sim, ConfiguredTeleOp.CLASS, 300, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;
            g1.left_stick_y = tick >= 40 && tick < 70 ? -0.5f : 0f; // drive toward the hive
            g1.dpad_right = tick >= 120 && tick < 123;               // pollen trim +50
        }, null);
        List<List<String>> sent = result.telemetry().transmissions();
        for (List<String> one : sent) {
            for (String line : one) {
                if (line.startsWith("Pollen shooter") && Double.isNaN(targetAtStart[0]) && lines[0]++ > 5) {
                    targetAtStart[0] = target(line);
                }
            }
        }

        // Expected from the plant's true distance, worked out here.
        double d = Math.hypot(RED_HIVE_X - sim.drivePlant.getX(), -CELL_Y - sim.drivePlant.getY());
        double table = 1500.0 + (d - 30.0) / 40.0 * 800.0;
        double shown = target(lastLine(result, "Pollen shooter"));
        assertEquals(table + 50.0, shown, MechanismConstants.SPEED_TABLE_UPDATE_STEP_RPM + 5.0,
                "target = table at the true distance + trim (distance " + d + " in)");
        double d0 = Math.hypot(RED_HIVE_X + 13.0, -CELL_Y + 63.0);
        assertEquals(1500.0 + (d0 - 30.0) / 40.0 * 800.0, targetAtStart[0], MechanismConstants.SPEED_TABLE_UPDATE_STEP_RPM + 5.0,
                "the start distance's rpm");
        assertTrue(targetAtStart[0] - (shown - 50.0) > 150.0, "closer in, the table asks for less");
        assertEquals(shown, sim.pollenShooterRpm(), 100.0, "the flywheel holds it");
        assertNotEquals(1600.0, shown, "no longer the fixed target");
    }

    private static double target(String shooterLine) {
        Matcher m = Pattern.compile("target=(-?[0-9.]+)").matcher(shooterLine);
        assertTrue(m.find(), "no target in " + shooterLine);
        return Double.parseDouble(m.group(1));
    }

    // ------------------------------------------------------------------ a real shot with lead

    /**
     * The point of lead, with the CAD shot geometry (ShotModel, under the same what-if shot the
     * autos are tested with): a pollen fired from a robot strafing past the hive at 8 in/s crosses
     * the cell's line where a shot from a standing robot does when the robot aims at
     * AimController.leadAimPoint, and well to the side when it aims straight at the cell. With
     * lead it also seats in the up cell in the physics hive. (The hive's cell is forgiving: at this
     * speed the unled shot still drops in, so the crossing point is the sharp test.)
     */
    @Test
    void aShotFiredWhileStrafingWithLeadCrossesTheCellWhereAStandingShotDoes() {
        ShotSweepTest.withShot(ZenithAutoMatchTest.SHOT_PITCH_DEG, ZenithAutoMatchTest.WHAT_IF_GRIP, () -> {
            double rpm = MechanismConstants.AUTO_POLLEN_RPM;
            double ballGround = ShotModel.exitSpeed(rpm) * Math.cos(Math.toRadians(ShotModel.POLLEN_PITCH_DEG));
            double x = RED_HIVE_X, y = -48.0, vx = 8.0, vy = 0.0;
            double straight = AimController.desiredHeading(x, y, RED_HIVE_X, -CELL_Y);
            double[] p = AimController.leadAimPoint(x, y, vx, vy, RED_HIVE_X, -CELL_Y, ballGround);
            double led = AimController.desiredHeading(x, y, p[0], p[1]);
            assertTrue(Math.abs(wrap(led - straight)) > Math.toRadians(3.0), "a real lead");
            assertTrue(Math.abs(wrap(led - straight)) < AimController.MAX_LEAD_RAD, "inside the clamp");

            double still = xAtCellLine(ShotModel.pollenLaunch(x, y, straight, rpm, 0.0, 0.0));
            double unled = xAtCellLine(ShotModel.pollenLaunch(x, y, straight, rpm, vx, vy));
            double withLead = xAtCellLine(ShotModel.pollenLaunch(x, y, led, rpm, vx, vy));
            assertEquals(still, withLead, 0.5, "with lead the ball crosses where a standing shot does");
            assertTrue(Math.abs(unled - still) > 4.0, String.format(
                    "without lead it drifts with the robot: %.1f in off", unled - still));
            assertEquals(BiobuzzHive.cellName(true, false), fireFrom(x, y, vx, vy, led, rpm),
                    "and the led shot seats in the up cell");
        });
    }

    /** Field x where a launched ball {x, y, z, vx, vy, vz} crosses the cell's y (no drag across the ground). */
    private static double xAtCellLine(double[] l) {
        double t = (-CELL_Y - l[1]) / l[4];
        return l[0] + l[3] * t;
    }

    private static String fireFrom(double x, double y, double vx, double vy, double heading, double rpm) {
        GamePieceTracker tracker = BiobuzzHiveTest.tracker(BiobuzzHive.body(true));
        tracker.setBodyMotion(BiobuzzHive.bodyName(true), BiobuzzHive.angleWithUp(false), 0.0);
        double[] l = ShotModel.pollenLaunch(x, y, heading, rpm, vx, vy);
        int id = tracker.addHeldPiece("pollen", BiobuzzHiveTest.R);
        tracker.launch(id, "shot", l[0], l[1], l[2], l[3], l[4], l[5]);
        for (double t = 0; t < 3.0; t += 0.02) tracker.advance(0.02);
        return tracker.getPiece(id).getSeatedCellName();
    }

    // ------------------------------------------------------------------ helpers

    interface TickProbe {
        void accept(int activeTick, BioHorsSim sim);
    }

    private static void startAt(BioHorsSim sim, double x, double y, double h) {
        sim.placeAt(x, y, h);
        OpMode.blackboard.put("biohors.alliance", "RED");
        OpMode.blackboard.put("biohors.pose", new double[] {x, y, h});
    }

    private static OpModeRunResult run(BioHorsSim sim, String opMode, int activeTicks, GamepadScript gamepads,
                                       TickProbe probe) {
        final OpModePhase[] phaseNow = {OpModePhase.INIT};
        final int[] tickNow = {0};
        return OpModeRunner.forTeamClass(opMode, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(activeTicks)
                .gamepads((phase, tick, g1, g2) -> {
                    phaseNow[0] = phase;
                    tickNow[0] = tick;
                    gamepads.apply(phase, tick, g1, g2);
                })
                .eachTick(dt -> {
                    sim.step(dt);
                    if (probe != null && phaseNow[0] == OpModePhase.ACTIVE) probe.accept(tickNow[0], sim);
                })
                .run();
    }

    /** The last telemetry line whose caption starts with {@code caption}. */
    private static String lastLine(OpModeRunResult result, String caption) {
        for (String line : result.telemetry().lastTransmission()) {
            if (line.startsWith(caption)) return line;
        }
        return "";
    }

    /** True if any transmission had a line starting with caption that contains text. */
    private static boolean anyLine(OpModeRunResult result, String caption, String text) {
        for (List<String> sent : result.telemetry().transmissions()) {
            for (String line : sent) if (line.startsWith(caption) && line.contains(text)) return true;
        }
        return false;
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}
