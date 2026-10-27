package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.AutoPoses;
import org.firstinspires.ftc.teamcode.auto.TwoTipStateMachineAuto;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.simloop.simclock.SimClockLoader;
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
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * The hand-written state machine auto (TwoTipStateMachineAuto) run as a BIOBUZZ match: the real
 * OpMode, Pedro follower and mechanisms against BioHorsSim and BiobuzzMatch, stopped at 30 s the
 * way the field stops it. RED twice (identical logs), BLUE once (the mirror scores the same).
 */
class StateMachineAutoMatchTest {

    static final double AUTO_PERIOD_S = 30.0;
    static final double TICK_S = OpModeRunner.DEFAULT_TICK_SECONDS;

    @AfterEach
    void clear() {
        OpMode.blackboard.clear();
    }

    static final class Run {
        BiobuzzMatch match;
        Path rlog;
        List<String> states = new ArrayList<>();
        List<Double> stateStarts = new ArrayList<>();
        String report;

        double doneAt() {
            int i = states.indexOf("DONE");
            return i < 0 ? Double.NaN : stateStarts.get(i);
        }
    }

    @Test
    void scoresSixteenAndTipsTwiceAsRedAndBlue() throws IOException {
        Run red1 = run("RED", "run1");
        Run red = run("RED", "");
        Run blue = run("BLUE", "");
        System.out.print(red.report + blue.report);

        assertEquals(16, red.match.scored(), "RED pollen scored in AUTO\n" + red.report);
        assertEquals(2, red.match.tips(), "RED tips\n" + red.report);
        for (double t : red.match.tipTimes()) {
            assertTrue(t + 4.0 <= 38.0, "tip at " + t + " s completes after TELEOP starts");
        }
        List<String> expected = new ArrayList<>();
        for (TwoTipStateMachineAuto.State s : TwoTipStateMachineAuto.State.values()) expected.add(s.name());
        assertEquals(expected, red.states, "every state ran, in order\n" + red.report);
        assertTrue(red.doneAt() < AUTO_PERIOD_S, "parked at " + red.doneAt() + " s\n" + red.report);

        assertTrue(red.match.unintendedContacts().isEmpty(), "contact: " + red.match.unintendedContacts() + "\n" + red.report);
        assertTrue(red.match.strafeFraction() < 0.20, String.format(Locale.ROOT,
                "strafing is %.0f %% of the driving\n%s", 100 * red.match.strafeFraction(), red.report));

        List<RlogDecodedCompare.Mismatch> diff = RlogDecodedCompare.compare(red1.rlog, red.rlog);
        assertTrue(diff.isEmpty(), "two RED runs logged differently: " + RlogDecodedCompare.summarise(diff));

        assertEquals(red.match.scored(), blue.match.scored(), "BLUE should score what RED scores\n" + blue.report);
        assertEquals(red.match.tips(), blue.match.tips(), "BLUE tips\n" + blue.report);
        assertTrue(blue.match.unintendedContacts().isEmpty(), "BLUE contact: " + blue.match.unintendedContacts());
        assertEquals(expected, blue.states, "BLUE states\n" + blue.report);
    }

    private Run run(String alliance, String suffix) throws IOException {
        File root = projectRoot();
        File assets = new File(root, "TeamCode/src/main/assets/autos");
        BioHorsSim sim = new BioHorsSim();
        boolean red = "RED".equals(alliance);
        BiobuzzMatch match = new BiobuzzMatch(sim, readJson(new File(assets, "biobuzz.field.json")),
                readJson(new File(assets, "robot.json")), red, TwoTipStateMachineAuto.PRELOAD);
        com.pedropathing.math.Pose start = AutoPoses.of(red ? org.firstinspires.ftc.teamcode.aim.Alliance.RED
                : org.firstinspires.ftc.teamcode.aim.Alliance.BLUE, AutoPoses.START);
        sim.placeAt(start.x(), start.y(), start.heading());

        String tag = "state-machine-" + alliance.toLowerCase(Locale.ROOT) + (suffix.isEmpty() ? "" : "-" + suffix);
        Path rlog = new File(root, "TeamCode/build/sim/" + tag + ".rlog").toPath();
        Files.createDirectories(rlog.getParent());

        Run r = new Run();
        r.match = match;
        r.rlog = rlog;
        double[] time = {0};
        OpMode.blackboard.clear();
        OpMode.blackboard.put("biohors.alliance", alliance);
        OpModeRunner.forTeamClass(TwoTipStateMachineAuto.class, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .tickSeconds(TICK_S)
                .initLoopTicks(25)
                .activeTicks((int) Math.round(AUTO_PERIOD_S / TICK_S))
                .eachTick(dt -> {
                    sim.step(dt);
                    match.step(dt);
                    time[0] += dt;
                    Object s = OpMode.blackboard.get(TwoTipStateMachineAuto.STATE_KEY);
                    if (s != null && (r.states.isEmpty() || !r.states.get(r.states.size() - 1).equals(s))) {
                        r.states.add((String) s);
                        r.stateStarts.add(time[0]);
                    }
                })
                .runLogged(BioHorsSim.ROBOT_CONFIG, rlog);

        StringBuilder rep = new StringBuilder(String.format(Locale.ROOT, "%n== state machine %s%n", alliance));
        for (int i = 0; i < r.states.size(); i++) {
            rep.append(String.format(Locale.ROOT, "  %-28s %6.2f s%n", r.states.get(i), r.stateStarts.get(i)));
        }
        rep.append(String.format(Locale.ROOT,
                "  scored %d (missed %d, launched %d, collected %d), tips %d at %s, held at end %d%n",
                match.scored(), match.missed(), match.launched(), match.collected(), match.tips(), match.tipTimes(),
                match.held()));
        rep.append(String.format(Locale.ROOT, "  strafe %.0f %% of %.0f in driven%n",
                100 * match.strafeFraction(), match.drivenIn()));
        rep.append("  contacts: ").append(match.contacts()).append('\n');
        rep.append("  misses: ").append(match.missLog()).append('\n');
        r.report = rep.toString();
        return r;
    }

    static Json readJson(File file) throws IOException {
        try (InputStream in = new FileInputStream(file)) {
            return Json.parse(in, file.getName());
        }
    }

    static File projectRoot() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) dir = dir.getParentFile();
        if (dir == null) throw new IllegalStateException("no zenith.json above " + System.getProperty("user.dir"));
        return dir;
    }
}
