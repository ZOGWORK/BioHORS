package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.horizon36596.simloop.opmode.GamepadScript;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.simclock.SimClockLoader;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.junit.jupiter.api.AfterEach;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.firstinspires.ftc.teamcode.teleop.MainHorsTeleOp;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * The MainHORS teleop, run unchanged in SimLoop with scripted gamepads.
 *
 * Every assertion is on the simulated WORLD (the drive plant's true pose, the motors' real power,
 * the gate servos' real position), not on what the teleop believes, except where the point is the
 * Driver Station telemetry.
 *
 * Expected aim headings are computed from the hive coordinates (aim/FieldTargets) here, in the
 * test, with plain atan2, so a sign error in AimController cannot also be in the expectation.
 */
class MainHorsTeleOpSimTest {

    static final String TELEOP = "org.firstinspires.ftc.teamcode.teleop.MainHorsTeleOp";
    static final double TICK_S = OpModeRunner.DEFAULT_TICK_SECONDS;
    /** Long enough for the Pinpoint to finish calibrating before PLAY. */
    static final int INIT_LOOP_TICKS = 25;

    // RED hive pivot and cell centres, from the field file (SPEC). Repeated here on purpose.
    static final double RED_HIVE_X = -12.75;
    static final double BLUE_HIVE_X = 12.75;
    static final double CELL_Y = 18.528 / 2.0;

    @AfterEach
    void clearBlackboard() {
        OpMode.blackboard.clear();
    }

    // ------------------------------------------------------------------ drive

    @Test
    void leftStickDrivesForwardAndThePoseFollows() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0); // facing +Y
        // 1 s full forward, then 1 s with the sticks centred so the robot is at rest when the
        // teleop's last telemetry is compared with the plant.
        OpModeRunResult result = run(sim, 100, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.left_stick_y = tick < 50 ? -1f : 0f;
        }, null);

        double movedY = sim.drivePlant.getY() - (-63.0);
        assertTrue(movedY > 20.0, "1 s of full forward should move the robot well up the field, moved " + movedY);
        assertEquals(-13.0, sim.drivePlant.getX(), 1.0, "forward must not drift sideways");
        assertEquals(Math.PI / 2.0, sim.drivePlant.getHeading(), Math.toRadians(2.0), "forward must not turn");
        assertBeliefMatchesTruth(result, sim);
    }

    @Test
    void rightBumperIsSlowMode() {
        BioHorsSim normal = new BioHorsSim();
        startAt(normal, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(normal, 25, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.left_stick_y = -1f;
        }, null);
        OpMode.blackboard.clear();

        BioHorsSim slow = new BioHorsSim();
        startAt(slow, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(slow, 25, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) {
                g1.left_stick_y = -1f;
                g1.right_bumper = true;
            }
        }, null);

        double ratio = (slow.drivePlant.getY() + 63.0) / (normal.drivePlant.getY() + 63.0);
        assertEquals(0.45, ratio, 0.08, "right bumper should drive at about 0.45x");
    }

    // ------------------------------------------------------------------ aim

    @Test
    void leftTriggerTurnsTheRobotToFaceTheRedSouthCell() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, -40.0, 0.0);
        OpModeRunResult result = run(sim, 150, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.left_trigger = 1f;
        }, null);

        double expected = Math.atan2(-CELL_Y - sim.drivePlant.getY(), RED_HIVE_X - sim.drivePlant.getX());
        assertAngleNear(expected, sim.drivePlant.getHeading(), 3.0, "should face the RED south cell");
        String shown = lastTelemetry(result);
        assertTrue(shown.contains("SOUTH"), "telemetry should name the target cell: " + shown);
        assertTrue(shown.contains("Aiming") && shown.contains("ON"), "telemetry should show aiming ON: " + shown);
    }

    @Test
    void inTheNorthHalfItAimsAtTheNorthCellAndADoesNotChangeTheTarget() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, 40.0, 0.0);
        OpModeRunResult result = run(sim, 150, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.a = tick >= 2 && tick < 6; // A trims nectar; it must not touch the target
            g1.left_trigger = 1f;
        }, null);

        double north = Math.atan2(CELL_Y - sim.drivePlant.getY(), RED_HIVE_X - sim.drivePlant.getX());
        double south = Math.atan2(-CELL_Y - sim.drivePlant.getY(), RED_HIVE_X - sim.drivePlant.getX());
        assertAngleNear(north, sim.drivePlant.getHeading(), 3.0, "north of the centre line it should face the north cell");
        assertTrue(Math.abs(wrap(sim.drivePlant.getHeading() - south)) > Math.toRadians(8.0),
                "and visibly not the south one");
        assertTrue(lastTelemetry(result).contains("NORTH"), "telemetry names the north cell");
    }

    @Test
    void pressingAOnTheCentreLineDoesNotChangeTheTarget() {
        // y = 1 is inside the 2 in hysteresis band, where the half rule keeps whatever cell is
        // current (SOUTH, RED's kickoff up cell). Nothing else would overwrite a toggle here.
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, 1.0, 0.0);
        final OpModePhase[] phaseNow = {OpModePhase.INIT};
        final List<String> targetsShown = new ArrayList<>();
        OpModeRunResult result = run(sim, 120, (phase, tick, g1, g2) -> {
            phaseNow[0] = phase;
            if (phase != OpModePhase.ACTIVE) return;
            g1.a = (tick >= 5 && tick < 9) || (tick >= 40 && tick < 44) || (tick >= 70 && tick < 74);
            g2.a = tick >= 90 && tick < 94; // the operator's A too
            g1.left_trigger = 1f;
        }, null);

        for (List<String> sent : result.telemetry().transmissions()) {
            for (String line : sent) if (line.startsWith("Target")) targetsShown.add(line);
        }
        assertFalse(targetsShown.isEmpty(), "telemetry should show the target");
        for (String t : targetsShown) {
            assertTrue(t.contains("SOUTH"), "pressing A must never change the target: " + t);
        }
        double south = Math.atan2(-CELL_Y - sim.drivePlant.getY(), RED_HIVE_X - sim.drivePlant.getX());
        assertAngleNear(south, sim.drivePlant.getHeading(), 3.0, "still aiming at the south cell");
    }

    @Test
    void crossingTheCentreLineWhileAimingSwitchesTheTarget() {
        BioHorsSim sim = new BioHorsSim();
        // South half, facing the south cell; then drive straight north past y = 0 while aiming.
        startAt(sim, "RED", -45.0, -20.0, Math.atan2(-CELL_Y + 20.0, RED_HIVE_X + 45.0));
        final OpModePhase[] phaseNow = {OpModePhase.INIT};
        final int[] tickNow = {0};
        final double[] headingWhileSouth = {Double.NaN};
        OpModeRunResult result = OpModeRunner.forTeamClass(TELEOP, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(250)
                .gamepads((phase, tick, g1, g2) -> {
                    phaseNow[0] = phase;
                    tickNow[0] = tick;
                    if (phase != OpModePhase.ACTIVE) return;
                    g1.left_trigger = 1f;
                    // Robot-frame stick that drives field-north whatever the heading: the test
                    // reads the true heading, as a driver would steer by eye.
                    double h = sim.drivePlant.getHeading();
                    double drive = tick >= 30 && tick < 110 ? 0.5 : 0.0;
                    g1.left_stick_y = (float) (-drive * Math.sin(h));   // forward component of +y
                    g1.left_stick_x = (float) (-drive * Math.cos(h));   // right component of +y
                })
                .eachTick(dt -> {
                    sim.step(dt);
                    if (phaseNow[0] == OpModePhase.ACTIVE && tickNow[0] == 29) {
                        headingWhileSouth[0] = sim.drivePlant.getHeading();
                    }
                })
                .run();

        assertTrue(sim.drivePlant.getY() > 10.0, "the robot should have driven north of the centre line, y = "
                + sim.drivePlant.getY());
        double south0 = Math.atan2(-CELL_Y + 20.0, RED_HIVE_X + 45.0);
        assertAngleNear(south0, headingWhileSouth[0], 3.0, "south of the line it holds the south cell");
        double north = Math.atan2(CELL_Y - sim.drivePlant.getY(), RED_HIVE_X - sim.drivePlant.getX());
        assertAngleNear(north, sim.drivePlant.getHeading(), 3.0, "after crossing it faces the north cell");
        assertTrue(lastTelemetry(result).contains("NORTH"), "telemetry names the north cell");
    }

    @Test
    void aimingLeavesTranslationWithTheDriver() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, -40.0, 0.0);
        // Aim alone for 1.5 s to lock on, then strafe at full stick for 1.5 s while still aiming,
        // with the right stick pushed the whole time (aim must ignore it). The aim error is
        // measured on the true pose every tick of the strafe, not at the end, so coasting after
        // STOP does not count.
        final OpModePhase[] phaseNow = {OpModePhase.INIT};
        final int[] tickNow = {0};
        final double[] worstErrDeg = {0.0};
        final double[] startXY = new double[2];
        final double[] lastCellY = {-CELL_Y};
        final int[] switchTick = {-1000};
        OpModeRunner.forTeamClass(TELEOP, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(150)
                .gamepads((phase, tick, g1, g2) -> {
                    phaseNow[0] = phase;
                    tickNow[0] = tick;
                    if (phase != OpModePhase.ACTIVE) return;
                    g1.left_trigger = 1f;
                    g1.right_stick_x = 1f;
                    g1.left_stick_x = tick >= 75 ? -1f : 0f;
                })
                .eachTick(dt -> {
                    sim.step(dt);
                    if (phaseNow[0] != OpModePhase.ACTIVE) return;
                    if (tickNow[0] == 75) {
                        startXY[0] = sim.drivePlant.getX();
                        startXY[1] = sim.drivePlant.getY();
                    }
                    // The target is the cell on the robot's half of the field; the strafe may cross
                    // the centre line, after which the robot gets 0.5 s to swing to the other cell.
                    double y = sim.drivePlant.getY();
                    double cellY = y > 2.0 ? CELL_Y : y < -2.0 ? -CELL_Y : lastCellY[0];
                    if (cellY != lastCellY[0]) switchTick[0] = tickNow[0];
                    lastCellY[0] = cellY;
                    if (tickNow[0] >= 90 && tickNow[0] - switchTick[0] > 25) { // strafe going, settled
                        double want = Math.atan2(cellY - y, RED_HIVE_X - sim.drivePlant.getX());
                        double err = Math.toDegrees(Math.abs(wrap(sim.drivePlant.getHeading() - want)));
                        worstErrDeg[0] = Math.max(worstErrDeg[0], err);
                    }
                })
                .run();

        double moved = Math.hypot(sim.drivePlant.getX() - startXY[0], sim.drivePlant.getY() - startXY[1]);
        assertTrue(moved > 10.0, "the robot should still translate while aiming, moved " + moved);
        assertTrue(worstErrDeg[0] <= 6.0, String.format(
                "aim should track the cell while strafing; worst error %.1f deg", worstErrDeg[0]));
    }

    @Test
    void aimFallsBackToManualWhenThePinpointFaults() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, -40.0, 0.0);
        OpModeRunResult result = run(sim, 75, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.left_trigger = 1f; // aim held, sticks centred
        }, s -> {
            if (s == 0) sim.pinpoint.forceFault(GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED);
        });

        assertEquals(0.0, sim.drivePlant.getHeading(), Math.toRadians(1.0),
                "with a faulted Pinpoint the robot must not rotate on its own");
        String shown = lastTelemetry(result);
        assertTrue(shown.contains("POSE INVALID"), "telemetry should warn: " + shown);
    }

    @Test
    void blueAllianceAimsAtTheBlueHive() {
        BioHorsSim sim = new BioHorsSim();
        // No auto ran: pick BLUE with dpad right during INIT. The default BLUE start is the
        // point mirror of RED's (MainHorsTeleOp.DEFAULT_START_*: the auto's park, back to the
        // west wall facing the field), so on BLUE: back to the east wall, facing west.
        // Strafe first (aim off) to make the aim do real work, then hold the trigger with the
        // sticks centred.
        double x0 = -MainHorsTeleOp.DEFAULT_START_X_IN, y0 = -MainHorsTeleOp.DEFAULT_START_Y_IN;
        sim.placeAt(x0, y0, MainHorsTeleOp.DEFAULT_START_HEADING_RAD + Math.PI);
        run(sim, 200, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.INIT_LOOP) g1.dpad_right = true;
            if (phase != OpModePhase.ACTIVE) return;
            g1.left_stick_x = tick < 40 ? -1f : 0f;
            g1.left_trigger = tick >= 60 ? 1f : 0f;
        }, null);
        assertTrue(Math.hypot(sim.drivePlant.getX() - x0, sim.drivePlant.getY() - y0) > 15.0,
                "the strafe should have moved the robot sideways");

        // The target is BLUE's cell on the robot's half of the field (FieldTargets.cellForHalf).
        double cellY = sim.drivePlant.getY() > 0 ? CELL_Y : -CELL_Y;
        double expected = Math.atan2(cellY - sim.drivePlant.getY(), BLUE_HIVE_X - sim.drivePlant.getX());
        assertAngleNear(expected, sim.drivePlant.getHeading(), 3.0, "BLUE should aim at the BLUE cell on its half");
    }

    // ------------------------------------------------------------------ mechanisms

    @Test
    void dpadDownSpinsBothShootersToTheirOwnTargetsAndDpadRightTrimsOnlyPollen() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        OpModeRunResult result = run(sim, 250, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;     // shooters on
            g2.dpad_right = tick >= 10 && tick < 13;  // operator trims pollen +50
        }, null);

        assertEquals(1650.0, sim.pollenShooterRpm(), 100.0, "pollen shooter should hold its trimmed target");
        assertEquals(1600.0, sim.nectarShooterRpm(), 100.0, "nectar shooter keeps its own target");
        String shown = lastTelemetry(result);
        assertTrue(shown.contains("target=1650"), "pollen target shown as 1650: " + shown);
        assertTrue(shown.contains("target=1600"), "nectar target shown as 1600: " + shown);
    }

    @Test
    void aDpadHeldFromAlliancePickThroughPlayDoesNotTrimTheShooter() {
        BioHorsSim sim = new BioHorsSim();
        sim.placeAt(13.0, 63.0, -Math.PI / 2.0);
        OpModeRunResult result = run(sim, 40, (phase, tick, g1, g2) -> {
            // Pick BLUE in INIT and keep holding dpad right for a while after PLAY.
            g1.dpad_right = phase == OpModePhase.INIT_LOOP || (phase == OpModePhase.ACTIVE && tick < 10);
        }, null);
        String shown = lastTelemetry(result);
        assertTrue(shown.contains("BLUE"), "dpad right in INIT picks BLUE: " + shown);
        assertTrue(shown.contains("target=1600"), "holding it through PLAY must not trim pollen: " + shown);
        assertFalse(shown.contains("target=1650"), "no +50 trim: " + shown);
    }

    @Test
    void eachShooterHasItsOwnTrim() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        OpModeRunResult result = run(sim, 250, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;                                  // both on
            g1.a = (tick >= 10 && tick < 13) || (tick >= 20 && tick < 23);         // nectar +50 twice
            g2.left_bumper = tick >= 30 && tick < 33;                              // nectar -50 (operator)
            g1.dpad_left = tick >= 40 && tick < 43;                                // pollen -50
        }, null);
        String shown = lastTelemetry(result);
        assertTrue(shown.contains("Pollen shooter : on=true target=1550"), "pollen trimmed alone: " + shown);
        assertTrue(shown.contains("Nectar shooter : on=true target=1650"), "nectar trimmed alone: " + shown);
        assertEquals(1550.0, sim.pollenShooterRpm(), 100.0, "pollen holds its own target");
        assertEquals(1650.0, sim.nectarShooterRpm(), 100.0, "nectar holds its own target");
    }

    @Test
    void shootersStartOffAndGatesStartClosed() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(sim, 50, GamepadScript.NONE, null);
        assertEquals(0.0, sim.pollenShooter.getPower(), 1e-9);
        assertEquals(0.0, sim.nectarShooter.getPower(), 1e-9);
        assertEquals(0.485, sim.pollenGatePlant.getPosition(), 0.02, "pollen gate closed");
        assertEquals(0.485, sim.nectarGatePlant.getPosition(), 0.02, "nectar gate closed");
    }

    @Test
    void bTogglesBothGatesAndDpadUpTogglesTheFlowerBar() {
        BioHorsSim open = new BioHorsSim();
        startAt(open, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(open, 50, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.b = tick >= 2 && tick < 5;
        }, null);
        assertEquals(0.67, open.pollenGatePlant.getPosition(), 0.02, "B opens the pollen gate");
        assertEquals(0.67, open.nectarGatePlant.getPosition(), 0.02, "B opens the nectar gate");
        OpMode.blackboard.clear();

        BioHorsSim twice = new BioHorsSim();
        startAt(twice, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(twice, 60, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.b = (tick >= 2 && tick < 5) || (tick >= 20 && tick < 23);
        }, null);
        assertEquals(0.485, twice.pollenGatePlant.getPosition(), 0.02, "B again closes it");
        OpMode.blackboard.clear();

        BioHorsSim x = new BioHorsSim();
        startAt(x, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(x, 50, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.dpad_up = true;
        }, null);
        assertEquals(0.485, x.pollenGatePlant.getPosition(), 0.02, "dpad up (flower bar) does not move the gates");
        assertEquals(0.485, x.nectarGatePlant.getPosition(), 0.02, "dpad up (flower bar) does not move the gates");
        assertEquals(MechanismConstants.FLOWER_BAR_DOWN, x.flowerLeftPlant.getPosition(), 0.02, "dpad up puts the bar down");
        assertEquals(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE, x.flowerRight.getDirection(),
                "the right bar servo is mounted mirrored");
        assertEquals(1.0 - MechanismConstants.FLOWER_BAR_DOWN, x.flowerRightPlant.getPosition(), 0.02,
                "the right servo goes down too (mirrored output)");
        OpMode.blackboard.clear();

        BioHorsSim xTwice = new BioHorsSim();
        startAt(xTwice, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(xTwice, 60, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_up = (tick >= 2 && tick < 5) || (tick >= 20 && tick < 23);
        }, null);
        assertEquals(MechanismConstants.FLOWER_BAR_STOWED, xTwice.flowerLeftPlant.getPosition(), 0.02,
                "dpad up again stows the bar");
        assertEquals(1.0 - MechanismConstants.FLOWER_BAR_STOWED, xTwice.flowerRightPlant.getPosition(), 0.02,
                "dpad up again stows the right side too");
        OpMode.blackboard.clear();

        BioHorsSim shoot = new BioHorsSim();
        startAt(shoot, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(shoot, 60, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_up = tick >= 2 && tick < 5;     // bar down
            g1.b = tick >= 20 && tick < 23;   // gates open: the bar lifts slightly
        }, null);
        assertEquals(MechanismConstants.FLOWER_BAR_SHOOTING, shoot.flowerLeftPlant.getPosition(), 0.02,
                "with the gates open the bar sits at its shooting position");
        assertEquals(1.0 - MechanismConstants.FLOWER_BAR_SHOOTING, shoot.flowerRightPlant.getPosition(), 0.02,
                "both sides at the shooting position");
    }

    @Test
    void rightTriggerIntakesAndYOuttakes() {
        // Powers are sampled during the run: at STOP the teleop rightly turns everything off.
        BioHorsSim in = new BioHorsSim();
        startAt(in, "RED", -13.0, -63.0, Math.PI / 2.0);
        double[] inPower = sampleIntake(in, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.right_trigger = 1f;
        });
        assertEquals(1.0, inPower[0], 1e-9, "right trigger runs intakeLeft in");
        assertEquals(1.0, inPower[1], 1e-9, "right trigger runs intakeRight in");
        OpMode.blackboard.clear();

        BioHorsSim out = new BioHorsSim();
        startAt(out, "RED", -13.0, -63.0, Math.PI / 2.0);
        double[] outPower = sampleIntake(out, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g2.y = true; // operator mirror
        });
        assertEquals(-1.0, outPower[0], 1e-9, "Y runs intakeLeft out");
        assertEquals(-1.0, outPower[1], 1e-9, "Y runs intakeRight out");
    }

    /** The intake powers seen on the last ACTIVE tick (before STOP turns them off). */
    private static double[] sampleIntake(BioHorsSim sim, GamepadScript gamepads) {
        double[] seen = new double[2];
        OpModeRunner.forTeamClass(TELEOP, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(20)
                .gamepads(gamepads)
                .eachTick(dt -> {
                    sim.step(dt);
                    if (sim.intakeLeft.getPower() != 0.0 || sim.intakeRight.getPower() != 0.0) {
                        seen[0] = sim.intakeLeft.getPower();
                        seen[1] = sim.intakeRight.getPower();
                    }
                })
                .run();
        return seen;
    }

    // ------------------------------------------------------------------ determinism + RLOG

    @Test
    void aTeleopRunIsDeterministicAndLogsForAdvantageScope() throws IOException {
        Path first = Paths.get("build", "sim", "teleop-aim-run1.rlog");
        Path second = Paths.get("build", "sim", "teleop-aim.rlog");
        loggedAimRun(first);
        OpMode.blackboard.clear();
        loggedAimRun(second);

        assertTrue(Files.size(second) > 0, "the RLOG should not be empty");
        List<RlogDecodedCompare.Mismatch> diff = RlogDecodedCompare.compare(first, second);
        assertTrue(diff.isEmpty(), "two identical runs should log identically: " + RlogDecodedCompare.summarise(diff));
    }

    private void loggedAimRun(Path rlog) {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, -40.0, 0.0);
        OpModeRunner.forTeamClass(TELEOP, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(150)
                .gamepads((phase, tick, g1, g2) -> {
                    if (phase != OpModePhase.ACTIVE) return;
                    g1.dpad_down = tick < 3;
                    g1.left_trigger = 1f;
                    g1.left_stick_x = tick > 75 ? -0.6f : 0f;
                })
                .eachTick(dt -> {
                    sim.step(dt);
                    logTruth(sim);
                })
                .runLogged(BioHorsSim.ROBOT_CONFIG, rlog);
    }

    // ------------------------------------------------------------------ helpers

    /** Puts the plant at a pose and tells the teleop (as an auto would) that it starts there. */
    /**
     * X resets the pose to the reset spot (backed flat against the west wall at the park spot, by
     * the loading zone, intake facing the field), wiping odometry drift. The robot really stands there; its
     * pose is started wrong (as drift would leave it); X puts it right. Both alliances.
     */
    @Test
    void xResetsThePoseToTheResetSpotOnEitherAlliance() {
        for (String alliance : new String[] {"RED", "BLUE"}) {
            double s = "RED".equals(alliance) ? 1 : -1;
            double rx = s * MainHorsTeleOp.RESET_X_IN, ry = s * MainHorsTeleOp.RESET_Y_IN;
            double rh = MainHorsTeleOp.RESET_HEADING_RAD + ("RED".equals(alliance) ? 0 : Math.PI);
            for (boolean pressX : new boolean[] {false, true}) {
                BioHorsSim sim = new BioHorsSim();
                sim.placeAt(rx, ry, rh);
                OpMode.blackboard.put("biohors.alliance", alliance);
                // What drift would leave: 5 in and 6 in off, and 9 deg turned.
                OpMode.blackboard.put("biohors.pose", new double[] {rx + 5.0, ry - 6.0, rh + Math.toRadians(9)});
                OpModeRunResult result = run(sim, 20, (phase, tick, g1, g2) -> {
                    if (phase == OpModePhase.ACTIVE) g1.x = pressX && tick >= 3 && tick < 6;
                }, null);
                double[] shown = shownPose(lastTelemetry(result));
                String what = alliance + (pressX ? " after X" : " without X");
                if (pressX) {
                    assertEquals(rx, shown[0], 0.5, what + ": x");
                    assertEquals(ry, shown[1], 0.5, what + ": y");
                    assertAngleNear(rh, Math.toRadians(shown[2]), 1.0, what + ": heading");
                } else {
                    assertTrue(Math.hypot(shown[0] - rx, shown[1] - ry) > 5.0, what + ": the drift should remain");
                }
                OpMode.blackboard.clear();
            }
        }
    }

    /** The x, y (in) and heading (deg) the teleop's "Pose" telemetry line shows. */
    private static double[] shownPose(String telemetry) {
        java.util.regex.Matcher m = java.util.regex.Pattern
                .compile("x=(-?[0-9.]+) y=(-?[0-9.]+) h=(-?[0-9.]+) deg").matcher(telemetry);
        assertTrue(m.find(), "telemetry should show the pose: " + telemetry);
        return new double[] {Double.parseDouble(m.group(1)), Double.parseDouble(m.group(2)), Double.parseDouble(m.group(3))};
    }

    private static void startAt(BioHorsSim sim, String alliance, double x, double y, double h) {
        sim.placeAt(x, y, h);
        OpMode.blackboard.put("biohors.alliance", alliance);
        OpMode.blackboard.put("biohors.pose", new double[] {x, y, h});
    }

    private static OpModeRunResult run(BioHorsSim sim, int activeTicks, GamepadScript gamepads,
                                       Consumer<Integer> beforeTick) {
        final int[] tickNo = {0};
        return OpModeRunner.forTeamClass(TELEOP, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(activeTicks)
                .gamepads((phase, tick, g1, g2) -> {
                    if (beforeTick != null && phase == OpModePhase.ACTIVE) beforeTick.accept(tickNo[0]++);
                    gamepads.apply(phase, tick, g1, g2);
                })
                .eachTick(sim::step)
                .run();
    }

    static void logTruth(BioHorsSim sim) {
        Logger.recordOutput("Field/Robot", org.psilynx.psikit.core.wpi.math.Pose2d.struct,
                AdvantageScopeFrame.pose2d(sim.drivePlant.getX(), sim.drivePlant.getY(), sim.drivePlant.getHeading()));
        Logger.recordOutput("Truth/xIn", sim.drivePlant.getX());
        Logger.recordOutput("Truth/yIn", sim.drivePlant.getY());
        Logger.recordOutput("Truth/headingRad", sim.drivePlant.getHeading());
        Logger.recordOutput("Shooter/pollenRpm", sim.pollenShooterRpm());
        Logger.recordOutput("Shooter/nectarRpm", sim.nectarShooterRpm());
        Logger.recordOutput("Gate/pollen", sim.pollenGatePlant.getPosition());
        Logger.recordOutput("Gate/nectar", sim.nectarGatePlant.getPosition());
    }

    /** The robot's belief (from its own Pose telemetry line) agrees with the plant's truth. */
    private static void assertBeliefMatchesTruth(OpModeRunResult result, BioHorsSim sim) {
        String pose = lastTelemetry(result);
        java.util.regex.Matcher m = java.util.regex.Pattern
                .compile("x=(-?[0-9.]+) y=(-?[0-9.]+)").matcher(pose);
        assertTrue(m.find(), "no pose in telemetry: " + pose);
        // The Pinpoint reads one tick behind the plant: allow one tick of travel.
        assertEquals(sim.drivePlant.getX(), Double.parseDouble(m.group(1)), 1.5, "belief x vs truth");
        assertEquals(sim.drivePlant.getY(), Double.parseDouble(m.group(2)), 1.5, "belief y vs truth");
        assertFalse(pose.contains("INVALID"), "pose should be valid: " + pose);
    }

    private static String lastTelemetry(OpModeRunResult result) {
        return String.join(" | ", result.telemetry().lastTransmission());
    }

    private static void assertAngleNear(double expected, double actual, double tolDeg, String what) {
        double err = Math.toDegrees(Math.abs(wrap(actual - expected)));
        assertTrue(err <= tolDeg, String.format("%s: expected %.1f deg, got %.1f deg (off by %.1f)",
                what, Math.toDegrees(expected), Math.toDegrees(actual), err));
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}
