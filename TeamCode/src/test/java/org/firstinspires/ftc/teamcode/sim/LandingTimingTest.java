package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Rule-mode scoring happens when a pollen lands, not when it leaves the robot. RED's south cell is up
 * at kickoff with 3 nectar staged, so the season table tips it at 3 pollen. The robot fires 5 from the
 * south scoring spot, one every LAUNCH_S_PER_PIECE (0.3 s): each scores FLIGHT_S after it leaves; the
 * 3rd tips the hive as it lands; the 4th lands 0.3 s after the tip, inside TIP_CATCH_S, and still
 * counts (spilling with the load); the 5th lands 0.6 s after it, outside, and misses. All read from
 * the log, frame by frame (one frame a tick).
 */
class LandingTimingTest {

    @AfterEach
    void clear() {
        OpMode.blackboard.clear();
    }

    @Test
    void aShotScoresWhenItLandsAndTheHiveTipsOnTheLanding() throws IOException {
        File root = projectRoot();
        File assets = new File(root, "TeamCode/src/main/assets/autos");
        BioHorsSim sim = new BioHorsSim();
        BiobuzzMatch match = new BiobuzzMatch(sim, readJson(new File(assets, "biobuzz.field.json")),
                readJson(new File(assets, "robot.json")), true, 5);
        sim.placeAt(org.firstinspires.ftc.teamcode.auto.AutoPoses.SCORE_SOUTH.x(),
                org.firstinspires.ftc.teamcode.auto.AutoPoses.SCORE_SOUTH.y(), Math.PI / 2);
        Path rlog = new File(root, "TeamCode/build/sim/landing-timing.rlog").toPath();
        Files.createDirectories(rlog.getParent());
        OpMode.blackboard.put(PhysicsTipDemoTest.ALLIANCE + "rpm", 1600.0);   // the team's test target
        OpModeRunner.forTeamClass(PhysicsTipDemoTest.ShootFromHere.class, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.")
                .keepBlackboard(true)
                .initLoopTicks(5)
                .activeTicks((int) Math.round(6.0 / OpModeRunner.DEFAULT_TICK_SECONDS))
                .eachTick(dt -> {
                    sim.step(dt);
                    match.step(dt);
                })
                .runLogged(BioHorsSim.ROBOT_CONFIG, rlog);

        // The frame at which each count first reached 1, 2, 3...
        List<Integer> launched = new ArrayList<>(), scored = new ArrayList<>(), tips = new ArrayList<>();
        int missedAtEnd = 0, frame = 0;
        for (Map<String, String> f : RlogDecodedCompare.decode(rlog)) {
            Integer l = count(f, "RealOutputs/Match/launched"), s = count(f, "RealOutputs/Match/scored"),
                    t = count(f, "RealOutputs/Match/tips"), m = count(f, "RealOutputs/Match/missed");
            if (l != null) while (launched.size() < l) launched.add(frame);
            if (s != null) while (scored.size() < s) scored.add(frame);
            if (t != null) while (tips.size() < t) tips.add(frame);
            if (m != null) missedAtEnd = m;
            frame++;
        }
        int flightTicks = (int) Math.round(BiobuzzMatch.FLIGHT_S / OpModeRunner.DEFAULT_TICK_SECONDS);
        assertEquals(5, launched.size(), "all 5 should leave the robot");
        assertEquals(4, scored.size(), "3 tip it, the 4th lands inside TIP_CATCH_S; scored at frames " + scored);
        assertEquals(1, missedAtEnd, "the 5th lands after the cell swung away");
        for (int i = 0; i < 4; i++) {
            int wait = scored.get(i) - launched.get(i);
            assertTrue(Math.abs(wait - flightTicks) <= 1,
                    "pollen " + (i + 1) + " scored " + wait + " ticks after it left, not FLIGHT_S (" + flightTicks + ")");
        }
        assertEquals(1, tips.size(), "one tip");
        assertEquals(scored.get(2), tips.get(0), "the hive tips as the 3rd pollen lands");
    }

    private static Integer count(Map<String, String> frame, String key) {
        for (Map.Entry<String, String> e : frame.entrySet()) {
            if (e.getKey().endsWith(key)) return (int) Math.round(Double.parseDouble(e.getValue()));
        }
        return null;
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
