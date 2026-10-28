package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;

/**
 * The physics working end to end, for AdvantageScope's 3D field: the robot parked 40 in south of
 * RED's hive, facing it, with 8 pollen aboard, spins the pollen shooter up with the real
 * ShooterController, opens the real gate and feeds. Each pollen is a real ball (ShotModel) and the
 * hive (BiobuzzHive) seats them, tips under the 8th and spills them onto the floor.
 *
 * The shot is the stated WHAT-IF (ZenithAutoMatchTest.WHAT_IF_*); the rpm is the lowest that seats
 * a ball from this spot under it (ShotSweepTest), plus 100: SET FROM SIM.
 *
 * RLOG: TeamCode/build/sim/physics-tip-demo.rlog (Field/Robot, Field/RobotComponents, Field/Pollen,
 * Field/Hive).
 */
class PhysicsTipDemoTest {

    static final double SPOT_Y_IN = -50.0;
    static final String ALLIANCE = "biohors.demo.";

    /** Spins up, then opens the gate and feeds once the shooter is at its target. */
    public static class ShootFromHere extends OpMode {
        private BioHorsRobot robot;

        @Override
        public void init() {
            robot = new BioHorsRobot(hardwareMap, telemetry);
            robot.pollenShooter.setTargetRpm((Double) blackboard.get(ALLIANCE + "rpm"));
            robot.pollenGate.setClosed(true);
        }

        @Override
        public void start() {
            robot.pollenShooter.setShooterOn(true);
        }

        @Override
        public void loop() {
            robot.updateMechanisms();
            if (robot.pollenShooter.isAtTarget()) {
                robot.pollenGate.setClosed(false);
                robot.intake.intake();
            }
        }

        @Override
        public void stop() {
            robot.stop();
        }
    }

    @AfterEach
    void clear() {
        OpMode.blackboard.clear();
    }

    @Test
    void eightRealShotsTipTheHiveAndItSpills() throws IOException {
        demo(true);
    }

    /** BLUE's hive is a physics body too: the point mirror of RED's demo, into its north cell. */
    @Test
    void eightRealShotsTipBluesHiveAndItSpills() throws IOException {
        demo(false);
    }

    private void demo(boolean red) throws IOException {
        OpMode.blackboard.clear();
        double savedPitch = ShotModel.POLLEN_PITCH_DEG, savedGrip = ShotModel.GRIP_EFFICIENCY;
        ShotModel.POLLEN_PITCH_DEG = ZenithAutoMatchTest.WHAT_IF_PITCH_DEG;
        ShotModel.GRIP_EFFICIENCY = ZenithAutoMatchTest.WHAT_IF_GRIP;
        try {
            int min = ShotSweepTest.minRpm(SPOT_Y_IN);
            assertTrue(min > 0, "no rpm seats a ball from y " + SPOT_Y_IN);
            double rpm = min + 100;

            File root = projectRoot();
            File assets = new File(root, "TeamCode/src/main/assets/autos");
            BioHorsSim sim = new BioHorsSim();
            BiobuzzMatch match = new BiobuzzMatch(sim, readJson(new File(assets, "biobuzz.field.json")),
                    readJson(new File(assets, "robot.json")), red, 8).usePhysics();
            double side = red ? 1 : -1;   // BLUE: the point mirror, (x, y, h) -> (-x, -y, h + pi)
            sim.placeAt(-12.75 * side, SPOT_Y_IN * side, Math.PI / 2 + (red ? 0 : Math.PI));
            Path rlog = new File(root, "TeamCode/build/sim/physics-tip-demo" + (red ? "" : "-blue") + ".rlog").toPath();
            Files.createDirectories(rlog.getParent());

            OpMode.blackboard.put(ALLIANCE + "rpm", rpm);
            OpModeRunner.forTeamClass(ShootFromHere.class, sim.hardwareMap)
                    .teamPackages("org.firstinspires.ftc.teamcode.")
                    .keepBlackboard(true)
                    .tickSeconds(ZenithAutoMatchTest.PHYSICS_TICK_S)
                    .initLoopTicks(5)
                    .activeTicks((int) Math.round(14.0 / ZenithAutoMatchTest.PHYSICS_TICK_S))
                    .eachTick(dt -> {
                        sim.step(dt);
                        match.step(dt);
                    })
                    .runLogged(BioHorsSim.ROBOT_CONFIG, rlog);

            int southFloor = 0;
            for (GamePiece p : match.shotPieces()) {
                if (p.getFieldZInches() < 4.0 && side * p.getFieldYInches() < -10.0) southFloor++;
            }
            System.out.printf(Locale.ROOT, (red ? "RED" : "BLUE") + " TIP DEMO rpm %.0f: launched %d, scored %d, tips %d at %s, "
                            + "missed %s, on the south floor %d%n", rpm, match.launched(), match.scored(),
                    match.tips(), match.tipTimes(), match.missLog(), southFloor);

            // The intake runs while it feeds, so a spilled ball that rolls under the mouth is taken
            // and fired again: every launch past the 8 aboard must be one the intake really took.
            assertEquals(8, match.launched() - match.collected(), "all 8 pollen aboard should leave the robot");
            assertEquals(0, match.held(), "the robot should end empty");
            // The up cell starts with its 3 staged nectar, so the 3rd seated pollen tips it (the season
            // table); the shots after that go into the swinging cell or miss once it has swung away.
            assertTrue(match.scored() >= 3, "at least the 3 that tip it should seat, got " + match.scored());
            assertEquals(1, match.tips(), "the cell tips once");
            // At least the 3 pollen that tipped it (with the staged nectar) come out on its own side.
            assertTrue(southFloor >= 3, "the tipped cell should spill its load on its own side, got " + southFloor);
            java.util.Set<String> keys = RlogDecodedCompare.decodedKeys(rlog);
            for (String k : new String[] {"Field/Robot", "Field/RobotComponents", "Field/Pollen", "Field/Hive"}) {
                assertTrue(keys.stream().anyMatch(x -> x.endsWith(k)), "RLOG has no " + k);
            }
        } finally {
            ShotModel.POLLEN_PITCH_DEG = savedPitch;
            ShotModel.GRIP_EFFICIENCY = savedGrip;
        }
    }

    private static Json readJson(File file) throws IOException {
        try (InputStream in = new FileInputStream(file)) {
            return Json.parse(in, file.getName());
        }
    }

    private static File projectRoot() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) dir = dir.getParentFile();
        if (dir == null) throw new IllegalStateException("no zenith.json above " + System.getProperty("user.dir"));
        return dir;
    }
}
