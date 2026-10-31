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
import org.firstinspires.ftc.teamcode.utilities.EndgameAlerts;
import org.firstinspires.ftc.teamcode.teleop.MainHorsTeleOp;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
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
        // Aiming spun the shooters up; on target and at speed, the fire interlock is clear.
        assertEquals("Fire : READY", line(result, "Fire"), "on the cell, the display says READY");
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
    }

    @Test
    void pressingAOnTheCentreLineDoesNotChangeTheTarget() {
        // y = 1 is inside the 2 in hysteresis band, where the half rule keeps whatever cell is
        // current (SOUTH, RED's kickoff up cell). Nothing else would overwrite a toggle here.
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, 1.0, 0.0);
        final double[] worstErrToSouthDeg = {0.0};
        run(sim, 120, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.a = (tick >= 5 && tick < 9) || (tick >= 40 && tick < 44) || (tick >= 70 && tick < 74);
            g2.a = tick >= 90 && tick < 94; // the operator's A too
            g1.left_trigger = 1f;
        }, tick -> {
            // From tick 60 on (aim settled, the first press long done), the TRUE heading must stay on
            // the south cell through every later press: a switched target would swing it ~37 deg.
            if (tick < 60) return;
            double southNow = Math.atan2(-CELL_Y - sim.drivePlant.getY(), RED_HIVE_X - sim.drivePlant.getX());
            worstErrToSouthDeg[0] = Math.max(worstErrToSouthDeg[0],
                    Math.toDegrees(Math.abs(wrap(sim.drivePlant.getHeading() - southNow))));
        });

        assertTrue(worstErrToSouthDeg[0] <= 3.0, String.format(
                "pressing A must never change the target: worst %.1f deg off the south cell", worstErrToSouthDeg[0]));
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
        OpModeRunner.forTeamClass(TELEOP, sim.hardwareMap)
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
        // Aiming does not spin up with the pose invalid, so both reasons are named.
        assertEquals("Fire : BLOCKED: POSE INVALID; NO SHOOTER ON", line(result, "Fire"), "the Fire line should warn");
        assertTrue(line(result, "Pose").contains("INVALID"), "the Pose line flags it: " + line(result, "Pose"));
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
        assertEquals(1650.0, shownTargetRpm(result, "Pollen"), 0.5, "pollen target shown as 1650");
        assertEquals(1600.0, shownTargetRpm(result, "Nectar"), 0.5, "nectar target shown as 1600");
    }

    @Test
    void aDpadHeldFromAlliancePickThroughPlayDoesNotTrimTheShooter() {
        BioHorsSim sim = new BioHorsSim();
        sim.placeAt(13.0, 63.0, -Math.PI / 2.0);
        OpModeRunResult result = run(sim, 40, (phase, tick, g1, g2) -> {
            // Pick BLUE in INIT and keep holding dpad right for a while after PLAY.
            g1.dpad_right = phase == OpModePhase.INIT_LOOP || (phase == OpModePhase.ACTIVE && tick < 10);
            g1.dpad_down = phase == OpModePhase.ACTIVE && tick >= 15 && tick < 18; // on, so the line shows the firing target
        }, null);
        // No auto ran, so the pose is the alliance's default start: BLUE's is RED's point mirror.
        double[] shown = shownPose(lastTelemetry(result));
        assertEquals(-MainHorsTeleOp.DEFAULT_START_X_IN, shown[0], 1.0, "dpad right in INIT picks BLUE: x");
        assertEquals(-MainHorsTeleOp.DEFAULT_START_Y_IN, shown[1], 1.0, "dpad right in INIT picks BLUE: y");
        assertEquals(1600.0, shownTargetRpm(result, "Pollen"), 0.5,
                "holding it through PLAY must not trim pollen (no +50)");
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
        assertFalse(line(result, "Pollen").endsWith("OFF"), "pollen on: " + line(result, "Pollen"));
        assertFalse(line(result, "Nectar").endsWith("OFF"), "nectar on: " + line(result, "Nectar"));
        assertEquals(1550.0, shownTargetRpm(result, "Pollen"), 0.5, "pollen trimmed alone");
        assertEquals(1650.0, shownTargetRpm(result, "Nectar"), 0.5, "nectar trimmed alone");
        assertEquals(1550.0, sim.pollenShooterRpm(), 100.0, "pollen holds its own target");
        assertEquals(1650.0, sim.nectarShooterRpm(), 100.0, "nectar holds its own target");
    }

    @Test
    void shootersStartOffAndGatesStartClosed() {
        // Idle is off by default (MechanismConstants.SHOOTER_IDLE_ENABLED): unasked, the shooters
        // stand still. With idle switched on they turn slowly instead.
        for (boolean idleOn : new boolean[] {false, true}) {
            BioHorsSim sim = new BioHorsSim();
            startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
            if (idleOn) OpMode.blackboard.put(ConfiguredTeleOp.SHOOTER_IDLE, true);
            double[] power = new double[2];
            runWatching(idleOn ? ConfiguredTeleOp.CLASS : TELEOP, sim, 50, GamepadScript.NONE, tick -> {
                if (tick == 45) {
                    power[0] = sim.pollenShooter.getPower();
                    power[1] = sim.nectarShooter.getPower();
                }
            });
            if (idleOn) {
                assertTrue(power[0] > 0.0, "idle on: pollen idles, not stopped");
                assertIdlePower(power[0], "pollen at idle");
                assertIdlePower(power[1], "nectar at idle");
            } else {
                assertEquals(0.0, power[0], 1e-9, "idle off: pollen stopped");
                assertEquals(0.0, power[1], 1e-9, "idle off: nectar stopped");
            }
            assertEquals(0.485, sim.pollenGatePlant.getPosition(), 0.02, "pollen gate closed");
            assertEquals(0.485, sim.nectarGatePlant.getPosition(), 0.02, "nectar gate closed");
            OpMode.blackboard.clear();
        }
    }

    @Test
    void bTogglesBothGatesAndDpadUpTogglesTheFlowerBar() {
        // Every servo is read on a late ACTIVE tick: at STOP the robot shuts the gates and stows the bar.
        BioHorsSim open = new BioHorsSim();
        startAt(open, "RED", -13.0, -63.0, Math.PI / 2.0);
        // The interlock needs a shooter on and at speed (NO SHOOTER ON otherwise): dpad down first.
        double[] openLate = servosAt(open, 200, 195, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;
            g1.b = tick >= 150 && tick < 153;
        });
        assertEquals(0.67, openLate[0], 0.02, "B opens the pollen gate");
        assertEquals(0.67, openLate[1], 0.02, "B opens the nectar gate");
        OpMode.blackboard.clear();

        BioHorsSim twice = new BioHorsSim();
        startAt(twice, "RED", -13.0, -63.0, Math.PI / 2.0);
        double[] twiceLate = servosAt(twice, 230, 225, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;
            g1.b = (tick >= 150 && tick < 153) || (tick >= 180 && tick < 183);
        });
        assertEquals(0.485, twiceLate[0], 0.02, "B again closes it");
        OpMode.blackboard.clear();

        BioHorsSim x = new BioHorsSim();
        startAt(x, "RED", -13.0, -63.0, Math.PI / 2.0);
        double[] xLate = servosAt(x, 50, 45, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.dpad_up = true;
        });
        assertEquals(0.485, xLate[0], 0.02, "dpad up (flower bar) does not move the gates");
        assertEquals(0.485, xLate[1], 0.02, "dpad up (flower bar) does not move the gates");
        assertEquals(MechanismConstants.FLOWER_BAR_DOWN, xLate[2], 0.02, "dpad up puts the bar down");
        assertEquals(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE, x.flowerRight.getDirection(),
                "the right bar servo is mounted mirrored");
        assertEquals(1.0 - MechanismConstants.FLOWER_BAR_DOWN, xLate[3], 0.02,
                "the right servo goes down too (mirrored output)");
        OpMode.blackboard.clear();

        BioHorsSim xTwice = new BioHorsSim();
        startAt(xTwice, "RED", -13.0, -63.0, Math.PI / 2.0);
        double[] xTwiceLate = servosAt(xTwice, 60, 55, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_up = (tick >= 2 && tick < 5) || (tick >= 20 && tick < 23);
        });
        assertEquals(MechanismConstants.FLOWER_BAR_STOWED, xTwiceLate[2], 0.02, "dpad up again stows the bar");
        assertEquals(1.0 - MechanismConstants.FLOWER_BAR_STOWED, xTwiceLate[3], 0.02,
                "dpad up again stows the right side too");
        OpMode.blackboard.clear();

        BioHorsSim shoot = new BioHorsSim();
        startAt(shoot, "RED", -13.0, -63.0, Math.PI / 2.0);
        double[] shootLate = servosAt(shoot, 200, 195, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_up = tick >= 2 && tick < 5;     // bar down
            g1.dpad_down = tick >= 2 && tick < 5;   // shooters on
            g1.b = tick >= 150 && tick < 153;       // gates open: the bar lifts slightly
        });
        assertEquals(MechanismConstants.FLOWER_BAR_SHOOTING, shootLate[2], 0.02,
                "with the gates open the bar sits at its shooting position");
        assertEquals(1.0 - MechanismConstants.FLOWER_BAR_SHOOTING, shootLate[3], 0.02,
                "both sides at the shooting position");
    }

    @Test
    void stopShutsTheGatesAndStowsTheBarMidVolley() {
        // The match ends mid-volley: gates open, bar at its shooting position. After STOP the
        // servos must be commanded shut and stowed, or pollen rolls out.
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        double[] late = servosAt(sim, 200, 195, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_up = tick >= 2 && tick < 5;
            g1.dpad_down = tick >= 2 && tick < 5;
            g1.b = tick >= 150 && tick < 153;
        });
        assertEquals(MechanismConstants.POLLEN_GATE_OPEN, late[0], 0.02, "mid-volley before STOP: pollen gate open");
        assertEquals(MechanismConstants.FLOWER_BAR_SHOOTING, late[2], 0.02, "and the bar at shooting");
        // What the robot last commanded, read from the servos (the plants need time to follow).
        assertEquals(MechanismConstants.POLLEN_GATE_CLOSED, sim.pollenGate.getPosition(), 1e-9, "STOP shuts the pollen gate");
        assertEquals(MechanismConstants.NECTAR_GATE_CLOSED, sim.nectarGate.getPosition(), 1e-9, "STOP shuts the nectar gate");
        assertEquals(MechanismConstants.FLOWER_BAR_STOWED, sim.flowerLeft.getPosition(), 1e-9, "STOP stows the bar");
    }

    /** Runs the teleop; the pollen gate, nectar gate, flower left and right plant positions at {@code atTick}. */
    private static double[] servosAt(BioHorsSim sim, int activeTicks, int atTick, GamepadScript gamepads) {
        double[] seen = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
        runWatching(sim, activeTicks, gamepads, tick -> {
            if (tick != atTick) return;
            seen[0] = sim.pollenGatePlant.getPosition();
            seen[1] = sim.nectarGatePlant.getPosition();
            seen[2] = sim.flowerLeftPlant.getPosition();
            seen[3] = sim.flowerRightPlant.getPosition();
        });
        return seen;
    }

    @Test
    void rightTriggerIntakesAndYNoLongerOuttakes() {
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
        // Y is the volley now: it may feed inward through open gates, but never runs the intake out.
        assertTrue(outPower[0] >= 0.0, "Y never runs intakeLeft out: " + outPower[0]);
        assertTrue(outPower[1] >= 0.0, "Y never runs intakeRight out: " + outPower[1]);
    }

    // ------------------------------------------------------------------ pose handoff

    @Test
    void anOldAutoPoseIsNotUsedAndAFreshOneIsTakenOnce() {
        // The robot really stands at the default start (the park spot). The blackboard holds a pose
        // 30 in away from a practice auto saved 10 minutes ago: the teleop must not use it.
        BioHorsSim stale = new BioHorsSim();
        stale.placeAt(MainHorsTeleOp.DEFAULT_START_X_IN, MainHorsTeleOp.DEFAULT_START_Y_IN,
                MainHorsTeleOp.DEFAULT_START_HEADING_RAD);
        org.horizon36596.simloop.sim.FakeTimer savedClock = new org.horizon36596.simloop.sim.FakeTimer();
        OpMode.blackboard.put("biohors.alliance", "RED");
        OpMode.blackboard.put("biohors.pose", new double[] {-20.0, 30.0, 1.0});
        OpMode.blackboard.put("biohors.poseTimer", new org.horizon36596.simloop.sim.SimElapsedTime(savedClock));
        savedClock.advance(600.0);
        OpModeRunResult result = run(stale, 10, GamepadScript.NONE, null);
        double[] shown = shownPose(lastTelemetry(result));
        assertEquals(MainHorsTeleOp.DEFAULT_START_X_IN, shown[0], 0.5, "stale pose ignored: default x");
        assertEquals(MainHorsTeleOp.DEFAULT_START_Y_IN, shown[1], 0.5, "stale pose ignored: default y");
        boolean said = false;
        for (List<String> sent : result.telemetry().transmissions()) {
            said |= lineIn(sent, "Start pose").contains("600 s old, not used");
        }
        assertTrue(said, "INIT says why the auto's pose was not used");
        OpMode.blackboard.clear();

        // A fresh handoff is used, and taken: the blackboard no longer holds it afterwards.
        BioHorsSim fresh = new BioHorsSim();
        startAt(fresh, "RED", -20.0, 30.0, 1.0);
        OpModeRunResult first = run(fresh, 10, GamepadScript.NONE, null);
        assertEquals(-20.0, shownPose(lastTelemetry(first))[0], 0.5, "a fresh auto pose is used");
        assertFalse(OpMode.blackboard.containsKey("biohors.pose"), "and taken once");
        assertEquals("RED", OpMode.blackboard.get("biohors.alliance"), "the alliance stays");
    }

    // ------------------------------------------------------------------ spin-up while aiming

    @Test
    void holdingAimSpinsTheShootersUpAndLettingGoStopsThem() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, -40.0, 0.0);
        double[] whileAiming = new double[2];
        double[] afterRelease = new double[2];
        runWatching(sim, 200, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.left_trigger = tick < 150 ? 1f : 0f; // no dpad down at all
        }, tick -> {
            if (tick == 149) {
                whileAiming[0] = sim.pollenShooterRpm();
                whileAiming[1] = sim.nectarShooterRpm();
            }
            if (tick == 195) {
                afterRelease[0] = sim.pollenShooter.getPower();
                afterRelease[1] = sim.nectarShooter.getPower();
            }
        });
        assertEquals(1600.0, whileAiming[0], 100.0, "aiming alone spins pollen up to its target");
        assertEquals(1600.0, whileAiming[1], 100.0, "and nectar to its own");
        assertIdlePower(afterRelease[0], "trigger released, no volley: pollen back to idle");
        assertIdlePower(afterRelease[1], "nectar back to idle");
    }

    @Test
    void dpadDownStillKeepsTheShootersOnAfterAiming() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, -40.0, 0.0);
        double[] afterRelease = {Double.NaN};
        runWatching(sim, 200, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;          // toggled on first
            g1.left_trigger = tick >= 20 && tick < 100 ? 1f : 0f;
        }, tick -> {
            if (tick == 195) afterRelease[0] = sim.pollenShooterRpm();
        });
        assertEquals(1600.0, afterRelease[0], 100.0, "the dpad-down toggle outlasts the aim");
    }

    @Test
    void aVolleyKeepsTheShootersUpAfterTheTriggerIsReleased() {
        // Already facing the RED south cell (the driving-display start). Aim, request the gates,
        // let go of the trigger mid-volley, then cancel with B.
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        double[] midVolley = new double[3];
        double[] afterCancel = new double[2];
        runWatching(sim, 260, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.left_trigger = tick < 150 ? 1f : 0f;
            g1.b = (tick >= 140 && tick < 143) || (tick >= 210 && tick < 213);
        }, tick -> {
            if (tick == 200) {
                midVolley[0] = sim.pollenGatePlant.getPosition();
                midVolley[1] = sim.pollenShooterRpm();
                midVolley[2] = sim.nectarShooterRpm();
            }
            if (tick == 255) {
                afterCancel[0] = sim.pollenGatePlant.getPosition();
                afterCancel[1] = sim.pollenShooter.getPower();
            }
        });
        assertEquals(MechanismConstants.POLLEN_GATE_OPEN, midVolley[0], 0.02, "the volley is still feeding");
        assertEquals(1600.0, midVolley[1], 100.0, "pollen still at speed with the trigger released");
        assertEquals(1600.0, midVolley[2], 100.0, "nectar too");
        assertEquals(MechanismConstants.POLLEN_GATE_CLOSED, afterCancel[0], 0.02, "B cancels the volley");
        assertIdlePower(afterCancel[1], "and with the gates shut the aim spin-up ends: back to idle");
    }

    /** A drag on both flywheels, as pollen passing through slows them. PLACEHOLDER (a test load). */
    static final double VOLLEY_DIP_TORQUE_FRACTION = -0.15;

    @Test
    void aSpeedDipAfterTheTriggerIsReleasedNeverFeedsStoppedShooters() {
        // Aim, request the gates, let go of the trigger mid-volley; then the pollen passing through
        // drags both flywheels below target, so the interlock shuts the gates. The request still
        // stands, so the shooters must stay on and the gates reopen only once they are back at speed.
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        double gateHalfway = (MechanismConstants.POLLEN_GATE_OPEN + MechanismConstants.POLLEN_GATE_CLOSED) / 2.0;
        int[] openWithShootersOff = {0};
        boolean[] closedDuringDip = {false};
        double[] atEnd = new double[2];
        runWatching(sim, 300, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.left_trigger = tick < 150 ? 1f : 0f;
            g1.b = tick >= 140 && tick < 143;
        }, tick -> {
            double drag = tick >= 170 && tick < 185 ? VOLLEY_DIP_TORQUE_FRACTION : 0.0;
            sim.pollenShooter.setExternalTorqueFraction(drag);
            sim.nectarShooter.setExternalTorqueFraction(drag);
            boolean gateOpen = sim.pollenGatePlant.getPosition() > gateHalfway;
            // The gate servo lags, so a brief close shows as the gate leaving full open.
            if (tick >= 170 && tick < 200
                    && sim.pollenGatePlant.getPosition() < MechanismConstants.POLLEN_GATE_OPEN - 0.01) {
                closedDuringDip[0] = true;
            }
            if (tick >= 150 && gateOpen && sim.pollenShooter.getPower() < 0.01
                    && sim.nectarShooter.getPower() < 0.01) {
                openWithShootersOff[0]++;
            }
            if (tick == 295) {
                atEnd[0] = sim.pollenGatePlant.getPosition();
                atEnd[1] = sim.pollenShooterRpm();
            }
        });
        assertTrue(closedDuringDip[0], "the dip should make the interlock shut the gates (else this tests nothing)");
        assertEquals(0, openWithShootersOff[0], "ticks with the gates open and both shooters off");
        assertEquals(MechanismConstants.POLLEN_GATE_OPEN, atEnd[0], 0.02, "back at speed, the volley resumes");
        assertEquals(1600.0, atEnd[1], 100.0, "with pollen at its target");
    }

    @Test
    void withThePoseInvalidAimingDoesNotSpinUp() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, -40.0, 0.0);
        double[] power = {Double.NaN};
        final int[] n = {0};
        runWatching(sim, 75, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            if (n[0]++ == 0) sim.pinpoint.forceFault(GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED);
            g1.left_trigger = 1f;
        }, tick -> {
            if (tick == 70) power[0] = sim.pollenShooter.getPower();
        });
        assertIdlePower(power[0], "the interlock would keep the gates shut, so no spin-up past idle");
    }

    // ------------------------------------------------------------------ dead shooter encoder

    @Test
    void aDeadEncoderIsFlaggedAndTheHealthyShooterFiresAlone() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);   // facing the south cell
        sim.pollenShooter.setMeasuredShaftSpeedFraction(0.0);  // the pollen encoder reads nothing
        double[] worstLatePower = {0.0};
        double[] gate = {Double.NaN, Double.NaN};
        OpModeRunResult result = runWatching(sim, 250, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;   // both on
            g1.b = tick >= 150 && tick < 153;        // and ask to fire
        }, tick -> {
            if (tick >= 100) worstLatePower[0] = Math.max(worstLatePower[0], Math.abs(sim.pollenShooter.getPower()));
            if (tick == 245) {
                gate[0] = sim.pollenGatePlant.getPosition();
                gate[1] = sim.nectarGatePlant.getPosition();
            }
        });
        assertTrue(line(result, "Pollen").endsWith("ENCODER? (no speed reading)"), line(result, "Pollen"));
        assertTrue(line(result, "Nectar").endsWith(" rpm OK"), "nectar is unaffected: " + line(result, "Nectar"));
        assertTrue(worstLatePower[0] < 0.4, String.format(
                "feedforward only, not a wound-up loop: pollen power %.2f", worstLatePower[0]));
        assertEquals(MechanismConstants.POLLEN_GATE_CLOSED, gate[0], 0.02,
                "no speed reading: the pollen gate stays shut, nothing feeds an unverified flywheel");
        assertEquals(MechanismConstants.NECTAR_GATE_OPEN, gate[1], 0.02,
                "limp mode: the faulted shooter is left out of the interlock and nectar fires alone");
        assertTrue(line(result, "Fire").contains("POLLEN FAULT: nectar only"), line(result, "Fire"));
    }

    // ------------------------------------------------------------------ battery and match cues

    @Test
    void aLowBatteryIsShownOnTheFireLine() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        sim.battery.setVoltage(11.0);
        OpModeRunResult result = run(sim, 100, GamepadScript.NONE, null);
        assertTrue(line(result, "Fire").endsWith(" | LOW BATTERY 11.0 V"), line(result, "Fire"));
        // (Every other test runs at the sim's 12.0 V and never shows it.)
    }

    /**
     * The endgame countdown on the Fire line, from PLAY on sim time. (The rumbles that go with it
     * are not visible through the sim's gamepads; EndgameAlertsTest covers when they fire.)
     */
    @Test
    void theFireLineCountsDownTheEndgame() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        int ticks = (int) Math.round(EndgameAlerts.TELEOP_S / TICK_S);
        int endgameTick = (int) Math.round((EndgameAlerts.TELEOP_S - EndgameAlerts.ENDGAME_S) / TICK_S);
        OpModeRunResult result = run(sim, ticks, GamepadScript.NONE, null);
        List<List<String>> driving = new java.util.ArrayList<>();
        for (List<String> sent : result.telemetry().transmissions()) {
            if (!lineIn(sent, "Loop").isEmpty()) driving.add(sent);
        }
        int firstShown = -1;
        for (int i = 0; i < driving.size(); i++) {
            if (lineIn(driving.get(i), "Fire").contains("ENDGAME")) {
                firstShown = i;
                break;
            }
        }
        assertEquals(endgameTick, firstShown, 2, "the countdown starts with the endgame");
        assertTrue(lineIn(driving.get(firstShown), "Fire").endsWith(" | ENDGAME 30 s"),
                lineIn(driving.get(firstShown), "Fire"));
        String last = lineIn(driving.get(driving.size() - 1), "Fire");
        assertTrue(last.endsWith(" | ENDGAME 0 s") || last.endsWith(" | ENDGAME 1 s"), last);
    }

    // ------------------------------------------------------------------ more guards and cues

    @Test
    void aBackwardsEncoderIsFlaggedAndDoesNotRunAway() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        sim.pollenShooter.setMeasuredShaftSpeedFraction(-1.0);   // the pollen encoder reads backwards
        double[] worstLatePower = {0.0};
        OpModeRunResult result = runWatching(sim, 200, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.dpad_down = tick >= 2 && tick < 5;
        }, tick -> {
            if (tick >= 100) worstLatePower[0] = Math.max(worstLatePower[0], Math.abs(sim.pollenShooter.getPower()));
        });
        assertTrue(line(result, "Pollen").endsWith("ENCODER REVERSED? (reads backwards)"), line(result, "Pollen"));
        assertTrue(line(result, "Nectar").endsWith(" rpm OK"), "nectar is unaffected: " + line(result, "Nectar"));
        assertTrue(worstLatePower[0] < 0.4, String.format(
                "feedforward only, not a loop pushing against a backwards reading: %.2f", worstLatePower[0]));
    }

    @Test
    void aForgottenGateRequestIsDroppedAndCannotFireLater() {
        // Shooters on and at speed, facing well away from the hive, B pressed and forgotten. Later the
        // driver turns onto the hive by hand: the old request must not fire.
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, -40.0, 0.0);   // about 45 deg off the south cell
        double gateHalfway = (MechanismConstants.POLLEN_GATE_OPEN + MechanismConstants.POLLEN_GATE_CLOSED) / 2.0;
        boolean[] opened = {false};
        boolean[] dropShown = {false};
        OpModeRunResult result = runWatching(sim, 450, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;
            g1.b = tick >= 160 && tick < 163;              // at speed by now; off target
            g1.left_trigger = tick >= 350 ? 1f : 0f;       // much later: aim onto the hive
        }, tick -> {
            if (sim.pollenGatePlant.getPosition() > gateHalfway) opened[0] = true;
        });
        for (List<String> sent : result.telemetry().transmissions()) {
            dropShown[0] |= lineIn(sent, "Fire").contains("B request dropped");
        }
        assertTrue(dropShown[0], "the Fire line says the request was dropped");
        assertFalse(opened[0], "the dropped request never opens the gates");
        assertEquals("Fire : READY", line(result, "Fire"), "on target and at speed: it WOULD have fired");
    }

    @Test
    void aPoseJumpIsInvalidUntilX() {
        BioHorsSim sim = new BioHorsSim();
        double rx = MainHorsTeleOp.RESET_X_IN, ry = MainHorsTeleOp.RESET_Y_IN;
        startAt(sim, "RED", rx, ry, MainHorsTeleOp.RESET_HEADING_RAD);
        List<String> poseAt = new java.util.ArrayList<>();
        OpModeRunResult result = runWatching(sim, 120, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.x = tick >= 80 && tick < 110;   // the driver holds X to reset
        }, tick -> {
            // A glitch: for one loop the odometry says the robot is 30 in away.
            if (tick == 20) sim.pinpoint.moveChassisTo((rx + 30.0) * 25.4, ry * 25.4, MainHorsTeleOp.RESET_HEADING_RAD);
        });
        boolean invalidShown = false;
        boolean blocked = false;
        int n = 0;
        for (List<String> sent : result.telemetry().transmissions()) {
            if (lineIn(sent, "Loop").isEmpty()) continue;
            n++;
            if (n > 30 && n < 75) {
                invalidShown |= lineIn(sent, "Pose").contains("INVALID: POSE JUMPED");
                blocked |= lineIn(sent, "Fire").contains("POSE INVALID");
            }
        }
        assertTrue(invalidShown, "after the jump the Pose line says so");
        assertTrue(blocked, "and the interlock blocks on it");
        assertFalse(line(result, "Pose").contains("INVALID"), "X put it right: " + line(result, "Pose"));
    }

    @Test
    void drivingThroughAPinpointOutageIsNotAPoseJump() {
        // The Pinpoint drops out for a second while the driver drives on by hand; when it comes back
        // the pose has moved on by that second's driving. That is not a jump.
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -40.0, -40.0, Math.PI / 2.0);   // facing +Y, room to drive north
        boolean[] everJumped = {false};
        double[] y = new double[2];
        OpModeRunResult result = runWatching(sim, 150, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.left_stick_y = tick >= 20 && tick < 90 ? -0.6f : 0f;   // drive north through the outage
            if (tick == 40) sim.pinpoint.forceFault(GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED);
            if (tick == 90) sim.pinpoint.clearFault();
        }, tick -> {
            if (tick == 40) y[0] = sim.drivePlant.getY();
            if (tick == 90) y[1] = sim.drivePlant.getY();
        });
        for (List<String> sent : result.telemetry().transmissions()) {
            everJumped[0] |= lineIn(sent, "Pose").contains("POSE JUMPED");
        }
        assertTrue(y[1] - y[0] > 10.0, "the robot really drove during the outage: " + (y[1] - y[0]) + " in");
        assertFalse(everJumped[0], "driving through the outage must not read as a jump");
        assertFalse(line(result, "Pose").contains("INVALID"), "valid again: " + line(result, "Pose"));
    }

    @Test
    void aSlowLoopIsShown() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        OpModeRunResult result = OpModeRunner.forTeamClass(TELEOP, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .tickSeconds(0.05)             // a 50 ms loop
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(40)
                .eachTick(sim::step)
                .run();
        assertTrue(line(result, "Fire").contains(" | SLOW LOOP 50 ms"), line(result, "Fire"));
        // (Every other test runs at the 20 ms tick and never shows it.)
    }

    @Test
    void initShowsThePreMatchCheck() {
        for (double volts : new double[] {12.0, 12.8}) {
            BioHorsSim sim = new BioHorsSim();
            startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
            sim.battery.setVoltage(volts);
            OpModeRunResult result = run(sim, 2, GamepadScript.NONE, null);
            List<String> init = null;
            for (List<String> sent : result.telemetry().transmissions()) if (!lineIn(sent, "Check").isEmpty()) init = sent;
            assertTrue(init != null, "INIT sends the check");
            String all = String.join(" | ", init);
            assertTrue(all.contains("OK Pinpoint READY"), all);
            assertTrue(all.contains("OK shooter encoders at rest"), all);
            assertTrue(all.contains("OK auto pose waiting"), all);
            if (volts < 12.5) {
                assertEquals("Check : FIX BEFORE PLAY", lineIn(init, "Check"));
                assertTrue(all.contains("!! battery 12.0 V"), all);
            } else {
                assertEquals("Check : ALL OK", lineIn(init, "Check"));
                assertTrue(all.contains("OK battery 12.8 V"), all);
            }
            OpMode.blackboard.clear();
        }
    }

    @Test
    void trimsCarryToTheNextTeleopAndBackClearsThem() {
        BioHorsSim first = new BioHorsSim();
        startAt(first, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(first, 30, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_right = tick >= 2 && tick < 5;    // pollen +50
            g1.a = tick >= 10 && tick < 13;           // nectar +50
        }, null);

        BioHorsSim second = new BioHorsSim();
        startAt(second, "RED", -13.0, -63.0, Math.PI / 2.0);
        OpModeRunResult next = run(second, 30, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.dpad_down = tick >= 2 && tick < 5;
        }, null);
        assertEquals(1650.0, shownTargetRpm(next, "Pollen"), 0.5, "the pollen trim carried over");
        assertEquals(1650.0, shownTargetRpm(next, "Nectar"), 0.5, "the nectar trim carried over");

        BioHorsSim third = new BioHorsSim();
        startAt(third, "RED", -13.0, -63.0, Math.PI / 2.0);
        OpModeRunResult cleared = run(third, 30, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.INIT_LOOP) g1.back = true;   // clear in INIT
            if (phase == OpModePhase.ACTIVE) g1.dpad_down = tick >= 2 && tick < 5;
        }, null);
        assertEquals(1600.0, shownTargetRpm(cleared, "Pollen"), 0.5, "back in INIT cleared it");
        assertEquals(1600.0, shownTargetRpm(cleared, "Nectar"), 0.5);
    }

    @Test
    void idlingShootersNeverOpenTheGates() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);   // on target
        double[] gate = {Double.NaN};
        OpMode.blackboard.put(ConfiguredTeleOp.SHOOTER_IDLE, true);   // idle is off by default
        OpModeRunResult result = runWatching(ConfiguredTeleOp.CLASS, sim, 250, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.b = tick >= 150 && tick < 153;   // idle reached long since
        }, tick -> {
            if (tick == 200) gate[0] = sim.pollenGatePlant.getPosition();
        });
        assertTrue(line(result, "Pollen").endsWith(" / 800 rpm IDLE"), line(result, "Pollen"));
        assertEquals(MechanismConstants.POLLEN_GATE_CLOSED, gate[0], 0.02, "idle is not ready to fire");
        assertTrue(line(result, "Fire").contains("NO SHOOTER ON"), line(result, "Fire"));
    }

    // ------------------------------------------------------------------ intake jam

    /** A jammed roller in the sim: the load takes 90 % of the motor's torque. PLACEHOLDER (a test load). */
    static final double JAM_TORQUE_FRACTION = -0.9;

    @Test
    void aStalledIntakeReversesBrieflyAndTheDisplaySaysSo() {
        for (boolean jam : new boolean[] {false, true}) {
            BioHorsSim sim = new BioHorsSim();
            startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
            int[] reverseTicks = {0};
            double[] powerAtEnd = {Double.NaN};
            OpModeRunResult result = runWatching(sim, 120, (phase, tick, g1, g2) -> {
                if (phase == OpModePhase.ACTIVE) g1.right_trigger = 1f;
            }, tick -> {
                if (jam && tick == 40) {
                    sim.intakeLeft.setExternalTorqueFraction(JAM_TORQUE_FRACTION);
                    sim.intakeRight.setExternalTorqueFraction(JAM_TORQUE_FRACTION);
                }
                if (jam && tick == 75) { // the jam clears (the reverse spat it out)
                    sim.intakeLeft.setExternalTorqueFraction(0.0);
                    sim.intakeRight.setExternalTorqueFraction(0.0);
                }
                if (sim.intakeLeft.getPower() < 0) reverseTicks[0]++;
                if (tick == 115) powerAtEnd[0] = sim.intakeLeft.getPower();
            });
            boolean jamShown = false;
            for (List<String> sent : result.telemetry().transmissions()) {
                jamShown |= lineIn(sent, "Fire").contains("INTAKE JAM");
            }
            if (!jam) {
                assertEquals(0, reverseTicks[0], "a free intake never reverses");
                assertFalse(jamShown, "and the display never says JAM");
            } else {
                assertEquals(MechanismConstants.INTAKE_JAM_REVERSE_S, reverseTicks[0] * TICK_S, 1.5 * TICK_S,
                        "one reverse of the set length");
                assertTrue(jamShown, "the Fire line says INTAKE JAM while it reverses");
            }
            assertEquals(1.0, powerAtEnd[0], 1e-9, (jam ? "jam cleared" : "free") + ": pulling in at the end");
            OpMode.blackboard.clear();
        }
    }

    // ------------------------------------------------------------------ Y volley

    /** What one teleop run did, tick by tick, as the robot commanded it (servo targets, motor powers). */
    static final class VolleyTrace {
        int pollenOpenTicks, nectarOpenTicks;
        int firstOpenTick = -1;
        int lastOpenTick = -1;
        int fedTicks;                  // intake at INTAKE_POWER
        boolean fedWhileShut;          // intake pushing with both gates shut, during the volley
        boolean intakeDuringSpinUp;    // intake running before any gate opened
        boolean openedWhileSlow;       // a gate commanded open while a flywheel's TRUE speed was far off
        boolean ySeqShown;
        double pollenPowerLate = Double.NaN;
        boolean pollenGateLate, nectarGateLate;
    }

    private static boolean commandedOpen(org.horizon36596.simloop.fakehardware.FakeServo gate, double openPos) {
        return Math.abs(gate.getPosition() - openPos) < 1e-9;
    }

    private static VolleyTrace volleyRun(BioHorsSim sim, int activeTicks, int lateTick, GamepadScript gamepads) {
        return volleyRun(sim, activeTicks, lateTick, false, gamepads);
    }

    /** Runs the teleop with {@code gamepads}; samples the end state at {@code lateTick}. With
     *  {@code pollenDead} the pollen flywheel's speed is not judged (its encoder is dead on purpose). */
    private static VolleyTrace volleyRun(BioHorsSim sim, int activeTicks, int lateTick, boolean pollenDead,
                                         GamepadScript gamepads) {
        VolleyTrace t = new VolleyTrace();
        OpModeRunResult result = runWatching(sim, activeTicks, gamepads, tick -> {
            boolean p = commandedOpen(sim.pollenGate, MechanismConstants.POLLEN_GATE_OPEN);
            boolean n = commandedOpen(sim.nectarGate, MechanismConstants.NECTAR_GATE_OPEN);
            if (p) t.pollenOpenTicks++;
            if (n) t.nectarOpenTicks++;
            if ((p || n) && t.firstOpenTick < 0) t.firstOpenTick = tick;
            if (p || n) t.lastOpenTick = tick;
            double intake = sim.intakeLeft.getPower();
            if (intake == 1.0) t.fedTicks++;
            if (t.firstOpenTick < 0 && intake != 0.0) t.intakeDuringSpinUp = true;
            boolean slow = (!pollenDead && Math.abs(sim.pollenShooterRpm() - 1600.0) > 150.0)
                    || Math.abs(sim.nectarShooterRpm() - 1600.0) > 150.0;
            if ((p || n) && slow) t.openedWhileSlow = true;
            if (tick == lateTick) {
                t.pollenPowerLate = sim.pollenShooter.getPower();
                t.pollenGateLate = p;
                t.nectarGateLate = n;
            }
        });
        for (List<String> sent : result.telemetry().transmissions()) {
            t.ySeqShown |= lineIn(sent, "Fire").contains("Y SEQ");
        }
        return t;
    }

    @Test
    void yFromIdleSpinsUpThenFiresOneWindowAndGoesBackToIdle() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);   // facing the south cell
        VolleyTrace t = volleyRun(sim, 300, 290, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.y = tick >= 5 && tick < 7;
            g1.right_trigger = tick >= 5 && tick < 30 ? 1f : 0f;   // locked out: must not feed the spin-up
        });
        assertTrue(t.firstOpenTick > 0, "the volley opened the gates");
        assertFalse(t.openedWhileSlow, "never open while a flywheel is off its speed");
        // Open only inside one fire window, and for most of it: the interlock may still shut the gates
        // for a few ticks (the spin-up overshoots the rpm tolerance just after it arrives).
        double windowS = org.firstinspires.ftc.teamcode.teleop.VolleySequence.FIRE_WINDOW_S;
        assertTrue((t.lastOpenTick - t.firstOpenTick + 1) * TICK_S <= windowS + TICK_S,
                "all the open ticks inside one window: ticks " + t.firstOpenTick + ".." + t.lastOpenTick);
        assertTrue(t.pollenOpenTicks * TICK_S >= 0.8 * windowS, "open for most of the window: " + t.pollenOpenTicks + " ticks");
        assertEquals(t.pollenOpenTicks, t.nectarOpenTicks, "both gates together");
        assertEquals(t.pollenOpenTicks, t.fedTicks, 1, "the intake feeds exactly while the gates are open");
        assertFalse(t.intakeDuringSpinUp, "the right trigger is locked out during the volley");
        assertTrue(t.ySeqShown, "the Fire line says Y SEQ");
        assertFalse(t.pollenGateLate, "gates shut after the window");
        assertIdlePower(t.pollenPowerLate, "no dpad down before: the shooters go back to off (or idle)");
    }

    @Test
    void theVolleyNeverOpensAGateTheInterlockBlocks() {
        // Off target: the window comes and goes with the gates shut.
        BioHorsSim off = new BioHorsSim();
        startAt(off, "RED", -40.0, -40.0, 0.0);   // facing +X, about 45 deg off the south cell
        VolleyTrace offT = volleyRun(off, 250, 245, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.y = tick >= 5 && tick < 7;
        });
        assertEquals(0, offT.pollenOpenTicks + offT.nectarOpenTicks, "aim off: no gate ever opens");
        assertEquals(0, offT.fedTicks, "and nothing is fed against shut gates");
        assertTrue(offT.ySeqShown);
        OpMode.blackboard.clear();

        // Pose invalid.
        BioHorsSim lost = new BioHorsSim();
        startAt(lost, "RED", -13.0, -63.0, Math.PI / 2.0);
        boolean[] faulted = {false};
        VolleyTrace lostT = volleyRun(lost, 250, 245, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            if (!faulted[0]) {
                lost.pinpoint.forceFault(GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED);
                faulted[0] = true;
            }
            g1.y = tick >= 5 && tick < 7;
        });
        assertEquals(0, lostT.pollenOpenTicks + lostT.nectarOpenTicks, "pose invalid: no gate ever opens");
    }

    @Test
    void aSecondYCancelsTheVolley() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        VolleyTrace t = volleyRun(sim, 200, 190, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.y = tick == 5 || tick == 8;   // press, release, press
        });
        assertEquals(0, t.pollenOpenTicks + t.nectarOpenTicks, "cancelled during spin-up: never fires");
        assertIdlePower(t.pollenPowerLate, "cancelled: back to off (or idle)");
    }

    @Test
    void bIsIgnoredDuringTheVolleyAndDpadShootersStayOnAfter() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        VolleyTrace t = volleyRun(sim, 260, 250, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;    // shooters on, at speed long before the volley
            g1.y = tick >= 150 && tick < 152;
            g1.b = tick >= 160 && tick < 163;        // inside the window: must not become a request
        });
        assertTrue(t.firstOpenTick >= 150, "the volley fired: " + t.firstOpenTick);
        assertFalse(t.pollenGateLate, "after the window the gates shut: B during the volley was ignored");
        assertTrue(t.pollenPowerLate > 0.2, String.format(
                "dpad down had them on before: still spinning at the firing target (power %.2f)", t.pollenPowerLate));
    }

    @Test
    void aStandingBRequestSurvivesTheVolleyEnding() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        VolleyTrace t = volleyRun(sim, 260, 250, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_down = tick >= 2 && tick < 5;
            g1.b = tick >= 140 && tick < 143;        // the driver's own request: gates open
            g1.y = tick >= 150 && tick < 152;        // a volley on top, ends about tick 200
        });
        assertTrue(t.pollenGateLate && t.nectarGateLate, "the driver's B request still holds the gates open");
    }

    @Test
    void withADeadPollenEncoderTheVolleyFiresNectarOnly() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        sim.pollenShooter.setMeasuredShaftSpeedFraction(0.0);
        VolleyTrace t = volleyRun(sim, 250, 245, true, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.y = tick >= 5 && tick < 7;
        });
        assertEquals(0, t.pollenOpenTicks, "the faulted pollen gate never opens");
        assertTrue(t.nectarOpenTicks > 0, "the healthy nectar shooter fires alone");
        assertFalse(t.openedWhileSlow, "nectar only once it is at speed");
    }

    // ------------------------------------------------------------------ flower bar speed cap

    @Test
    void theBarDownCapsTheDriveSpeedAndSaysSo() {
        BioHorsSim stowed = new BioHorsSim();
        startAt(stowed, "RED", -13.0, -63.0, Math.PI / 2.0);
        run(stowed, 30, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.left_stick_y = tick >= 5 ? -1f : 0f;
        }, null);
        OpMode.blackboard.clear();

        BioHorsSim down = new BioHorsSim();
        startAt(down, "RED", -13.0, -63.0, Math.PI / 2.0);
        double[] barAtEnd = {Double.NaN};
        OpModeRunResult result = runWatching(down, 30, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.dpad_up = tick < 2;                      // bar down before the sprint starts
            g1.left_stick_y = tick >= 5 ? -1f : 0f;     // full stick, as if into the hive wall
        }, tick -> {
            if (tick == 28) barAtEnd[0] = down.flowerLeft.getPosition();
        });

        double ratio = (down.drivePlant.getY() + 63.0) / (stowed.drivePlant.getY() + 63.0);
        assertEquals(MainHorsTeleOp.BAR_DOWN_SPEED_CAP, ratio, 0.08, "bar down drives at about the cap");
        assertTrue(line(result, "Fire").contains("BAR DOWN: slow"), line(result, "Fire"));
        assertEquals(MechanismConstants.FLOWER_BAR_DOWN, barAtEnd[0], 1e-9,
                "auto-stow is off by default: a full-stick sprint leaves the bar down");
    }

    @Test
    void withAutoStowOnASustainedSprintStowsTheBarButSlowModeDoesNot() {
        for (boolean slowMode : new boolean[] {false, true}) {
            BioHorsSim sim = new BioHorsSim();
            startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
            OpMode.blackboard.put(ConfiguredTeleOp.BAR_AUTO_STOW, true);
            double[] bar = {Double.NaN, Double.NaN};
            OpModeRunResult result = runWatching(ConfiguredTeleOp.CLASS, sim, 80, (phase, tick, g1, g2) -> {
                if (phase != OpModePhase.ACTIVE) return;
                g1.dpad_up = tick >= 2 && tick < 5;
                g1.left_stick_y = tick >= 30 ? -1f : 0f;
                g1.right_bumper = slowMode;
            }, tick -> {
                if (tick == 30 + 10) bar[0] = sim.flowerLeft.getPosition();   // 0.2 s into the sprint
                if (tick == 75) bar[1] = sim.flowerLeft.getPosition();
            });
            assertEquals(MechanismConstants.FLOWER_BAR_DOWN, bar[0], 1e-9, "not before BAR_AUTO_STOW_S of sprinting");
            if (slowMode) {
                assertEquals(MechanismConstants.FLOWER_BAR_DOWN, bar[1], 1e-9, "slow mode held: never stowed");
            } else {
                assertEquals(MechanismConstants.FLOWER_BAR_STOWED, bar[1], 1e-9, "a sustained sprint stows the bar");
                boolean shown = false;
                for (List<String> sent : result.telemetry().transmissions()) {
                    shown |= lineIn(sent, "Fire").contains("BAR STOWED (sprint)");
                }
                assertTrue(shown, "the Fire line says why the bar went up");
            }
            OpMode.blackboard.clear();
        }
    }

    // ------------------------------------------------------------------ jam give-up

    @Test
    void aJamThatDoesNotClearStopsTheIntakeUntilTheTriggerIsPulledAgain() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);
        int[] reverseTicks = {0};
        double[] power = {Double.NaN, Double.NaN};
        OpModeRunResult result = runWatching(sim, 270, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.right_trigger = tick < 235 || tick >= 240 ? 1f : 0f;
        }, tick -> {
            if (tick == 40) {   // a ball wedges for good
                sim.intakeLeft.setExternalTorqueFraction(JAM_TORQUE_FRACTION);
                sim.intakeRight.setExternalTorqueFraction(JAM_TORQUE_FRACTION);
            }
            if (tick == 236) {  // the driver clears it by hand with the trigger released
                sim.intakeLeft.setExternalTorqueFraction(0.0);
                sim.intakeRight.setExternalTorqueFraction(0.0);
            }
            if (sim.intakeLeft.getPower() < 0) reverseTicks[0]++;
            if (tick == 230) power[0] = sim.intakeLeft.getPower();
            if (tick == 265) power[1] = sim.intakeLeft.getPower();
        });
        boolean shown = false;
        for (List<String> sent : result.telemetry().transmissions()) {
            shown |= lineIn(sent, "Fire").contains("INTAKE JAMMED: clear by hand");
        }
        assertEquals(MechanismConstants.INTAKE_JAM_GIVE_UP_REVERSES * MechanismConstants.INTAKE_JAM_REVERSE_S,
                reverseTicks[0] * TICK_S, 3 * TICK_S, "exactly the give-up count of reverses, then no more");
        assertEquals(0.0, power[0], 1e-9, "given up: the intake is stopped, not grinding the ball");
        assertTrue(shown, "the Fire line says to clear it by hand");
        assertEquals(1.0, power[1], 1e-9, "released and pulled again: intaking again");
    }

    @Test
    void aYVolleyDoesNotClearAJamGiveUpWhileTheTriggerIsHeld() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0);   // on target, so the volley does fire
        double[] power = {Double.NaN, Double.NaN};
        boolean[] fedAfterGiveUp = {false};
        OpModeRunResult result = runWatching(sim, 400, (phase, tick, g1, g2) -> {
            if (phase != OpModePhase.ACTIVE) return;
            g1.right_trigger = 1f;                  // held the whole time
            g1.y = tick >= 235 && tick < 237;       // a volley after the guard has given up
        }, tick -> {
            if (tick == 40) {
                sim.intakeLeft.setExternalTorqueFraction(JAM_TORQUE_FRACTION);
                sim.intakeRight.setExternalTorqueFraction(JAM_TORQUE_FRACTION);
            }
            if (tick == 230) power[0] = sim.intakeLeft.getPower();
            if (tick > 230 && sim.intakeLeft.getPower() != 0.0) fedAfterGiveUp[0] = true;
            if (tick == 390) power[1] = sim.intakeLeft.getPower();
        });
        assertEquals(0.0, power[0], 1e-9, "given up before the volley");
        assertFalse(fedAfterGiveUp[0], "neither the volley nor the held trigger restarts a given-up intake");
        assertEquals(0.0, power[1], 1e-9, "still stopped after the volley");
        assertTrue(line(result, "Fire").contains("INTAKE JAMMED: clear by hand"), line(result, "Fire"));
    }

    // ------------------------------------------------------------------ X hold

    @Test
    void aTapOfXDoesNotResetThePoseMidField() {
        // Mid-field, well away from the reset spot: a fat-fingered X tap must leave the pose alone.
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -20.0, -20.0, 0.0);
        OpModeRunResult result = run(sim, 45, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.x = tick >= 3 && tick < 15;   // 0.24 s, under RESET_HOLD_S
        }, null);
        double[] shown = shownPose(lastTelemetry(result));
        assertEquals(-20.0, shown[0], 1.0, "x unchanged by a tap");
        assertEquals(-20.0, shown[1], 1.0, "y unchanged by a tap");
    }

    /**
     * A shooter at idle power: under the ~0.27 a 1600 rpm firing target needs (the sim's free speed
     * is 6000 rpm), so a spun-up shooter fails this; idle (800 rpm) needs about 0.13.
     */
    static void assertIdlePower(double power, String what) {
        assertTrue(power < 0.2, String.format("%s: power %.3f", what, power));
    }

    /** Runs the teleop; {@code afterTick} sees each ACTIVE tick's number after the world moved. */
    private static OpModeRunResult runWatching(BioHorsSim sim, int activeTicks, GamepadScript gamepads,
                                               Consumer<Integer> afterTick) {
        return runWatching(TELEOP, sim, activeTicks, gamepads, afterTick);
    }

    private static OpModeRunResult runWatching(String opMode, BioHorsSim sim, int activeTicks, GamepadScript gamepads,
                                               Consumer<Integer> afterTick) {
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
                    if (phaseNow[0] == OpModePhase.ACTIVE) afterTick.accept(tickNow[0]);
                })
                .run();
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

    // ------------------------------------------------------------------ driving display

    static final List<String> DRIVING_CAPTIONS = List.of("Fire", "Pose", "Pollen", "Nectar", "Loop");

    @Test
    void theDrivingDisplayIsFiveLinesWithTheLoopTime() {
        BioHorsSim sim = new BioHorsSim();
        startAt(sim, "RED", -13.0, -63.0, Math.PI / 2.0); // faces the RED south cell already
        OpModeRunResult result = run(sim, 250, (phase, tick, g1, g2) -> {
            if (phase == OpModePhase.ACTIVE) g1.dpad_down = tick >= 2 && tick < 5; // shooters on
        }, null);

        List<List<String>> driving = new java.util.ArrayList<>();
        for (List<String> sent : result.telemetry().transmissions()) {
            if (!lineIn(sent, "Loop").isEmpty()) driving.add(sent);
        }
        assertTrue(driving.size() > 200, "every driving loop sends the display, got " + driving.size());

        List<String> first = driving.get(0);
        assertEquals(DRIVING_CAPTIONS, captions(first), "first driving screen: " + first);
        assertTrue(lineIn(first, "Pollen").endsWith(" rpm OFF"), "shooters start off (idle is off): " + first);
        assertTrue(lineIn(first, "Nectar").endsWith(" rpm OFF"), "shooters start off (idle is off): " + first);

        boolean spinningUpShown = false;
        for (List<String> sent : driving) spinningUpShown |= lineIn(sent, "Pollen").endsWith(" rpm SPINNING UP");
        assertTrue(spinningUpShown, "between OFF and OK the pollen line says SPINNING UP");

        List<String> last = result.telemetry().lastTransmission();
        System.out.println("DRIVING DISPLAY\n  " + String.join("\n  ", last));
        assertEquals(DRIVING_CAPTIONS, captions(last), "exactly the five lines, in order: " + last);
        assertEquals("Fire : READY", lineIn(last, "Fire"));
        assertTrue(lineIn(last, "Pollen").endsWith(" / 1600 rpm OK"), lineIn(last, "Pollen"));
        assertTrue(lineIn(last, "Nectar").endsWith(" / 1600 rpm OK"), lineIn(last, "Nectar"));
        assertFalse(lineIn(last, "Pose").contains("INVALID"), lineIn(last, "Pose"));

        // The sim ticks at 20 ms, so the last loop, the average and the peak are all one tick.
        java.util.regex.Matcher m = java.util.regex.Pattern
                .compile("Loop : ([0-9.]+) ms \\(avg ([0-9.]+), max ([0-9.]+)\\)").matcher(lineIn(last, "Loop"));
        assertTrue(m.find(), "loop line format: " + lineIn(last, "Loop"));
        double tickMs = TICK_S * 1000.0;
        assertEquals(tickMs, Double.parseDouble(m.group(1)), 0.5, "last loop");
        assertEquals(tickMs, Double.parseDouble(m.group(2)), 0.5, "rolling average");
        assertEquals(tickMs, Double.parseDouble(m.group(3)), 0.5, "peak: the INIT-to-PLAY gap is not counted");
    }

    private static List<String> captions(List<String> transmission) {
        List<String> out = new java.util.ArrayList<>();
        for (String l : transmission) out.add(caption(l));
        return out;
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
                OpMode.blackboard.put("biohors.poseTimer", new org.horizon36596.simloop.sim.SimElapsedTime(new org.horizon36596.simloop.sim.FakeTimer())); // saved just now
                OpModeRunResult result = run(sim, 45, (phase, tick, g1, g2) -> {
                    if (phase == OpModePhase.ACTIVE) g1.x = pressX && tick >= 3 && tick < 33;   // held 0.6 s
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
        OpMode.blackboard.put("biohors.poseTimer", new org.horizon36596.simloop.sim.SimElapsedTime(new org.horizon36596.simloop.sim.FakeTimer())); // saved just now
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

    /** The last transmission's line with this caption, or "" if there is none. */
    static String line(OpModeRunResult result, String caption) {
        return lineIn(result.telemetry().lastTransmission(), caption);
    }

    static String lineIn(List<String> transmission, String caption) {
        for (String l : transmission) if (caption(l).equals(caption)) return l;
        return "";
    }

    /** A telemetry line's caption: the text before " : ". */
    static String caption(String line) {
        int at = line.indexOf(" : ");
        return at < 0 ? line : line.substring(0, at);
    }

    /** The target rpm the shooter line shows ({@code Pollen : 1580 / 1600 rpm OK}). */
    static double shownTargetRpm(OpModeRunResult result, String shooter) {
        return targetRpmIn(line(result, shooter));
    }

    static double targetRpmIn(String shooterLine) {
        java.util.regex.Matcher m = java.util.regex.Pattern
                .compile(" : (-?[0-9.]+) / (-?[0-9.]+) rpm ").matcher(shooterLine);
        assertTrue(m.find(), "no current / target rpm in: " + shooterLine);
        return Double.parseDouble(m.group(2));
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
