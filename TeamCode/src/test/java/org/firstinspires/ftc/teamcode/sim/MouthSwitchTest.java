package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Which intake takes a pollen depends on the flower bar: with it stowed, only the front roller's mouth
 * (robot.json, out to 10.2 in); with it down, the flower mech's opening out to the bar tip (14.19 in).
 * A pollen 12.5 in ahead of a robot standing still with the intake running is out of the roller's
 * reach and inside the bar's, in both the rule-based and the physics model.
 */
class MouthSwitchTest {

    static final double BALL_AHEAD_IN = 12.5;   // SET BY HAND: past 10.2 + 1.4 (the roller's reach), short of 14.19
    static final String BAR = "biohors.test.barDown";

    /** Holds the bar where the blackboard says and runs the intake inward. */
    public static class Hold extends OpMode {
        private BioHorsRobot robot;

        @Override
        public void init() {
            robot = new BioHorsRobot(hardwareMap, telemetry);
        }

        @Override
        public void loop() {
            robot.flowerBar.setDown((Boolean) blackboard.get(BAR));
            robot.intake.intake();
            robot.updateMechanisms();
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
    void theStowedRollerCannotReachIt() throws IOException {
        assertEquals(0, collected(false, false), "rule mode, bar stowed");
        assertEquals(0, collected(false, true), "physics, bar stowed");
    }

    @Test
    void theLoweredBarTakesIt() throws IOException {
        assertEquals(1, collected(true, false), "rule mode, bar down");
        assertEquals(1, collected(true, true), "physics, bar down");
    }

    private static int collected(boolean barDown, boolean physics) throws IOException {
        File root = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (root != null && !new File(root, "zenith.json").isFile()) root = root.getParentFile();
        File assets = new File(root, "TeamCode/src/main/assets/autos");
        BioHorsSim sim = new BioHorsSim();
        BiobuzzMatch match = new BiobuzzMatch(sim, readJson(new File(assets, "biobuzz.field.json")),
                readJson(new File(assets, "robot.json")), true, 0);
        if (physics) match.usePhysics();
        sim.placeAt(0, -40, 0);   // open floor, facing +X
        match.addLoosePollen(BALL_AHEAD_IN, -40);
        Path rlog = new File(root, "TeamCode/build/sim/mouth-switch.rlog").toPath();
        Files.createDirectories(rlog.getParent());
        OpMode.blackboard.put(BAR, barDown);
        OpModeRunner.forTeamClass(Hold.class, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.")
                .keepBlackboard(true)
                .initLoopTicks(5)
                .activeTicks((int) Math.round(2.0 / OpModeRunner.DEFAULT_TICK_SECONDS))
                .eachTick(dt -> {
                    sim.step(dt);
                    match.step(dt);
                })
                .runLogged(BioHorsSim.ROBOT_CONFIG, rlog);
        return match.collected();
    }

    private static Json readJson(File file) throws IOException {
        try (InputStream in = new FileInputStream(file)) {
            return Json.parse(in, file.getName());
        }
    }
}
