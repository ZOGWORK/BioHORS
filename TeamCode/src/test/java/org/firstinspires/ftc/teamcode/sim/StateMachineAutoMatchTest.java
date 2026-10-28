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
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;

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
    static final double MAX_STRAFE_FRACTION = 0.35;

    @AfterEach
    void clear() {
        OpMode.blackboard.clear();
    }

    static final double PARKED_WITHIN_IN = 2.0;   // SET BY HAND: the follower's end tolerance, with margin

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
    void scoresTwelveAndTipsTwiceAsRedAndBlue() throws IOException {
        Run red1 = run("RED", "run1");
        Run red = run("RED", "");
        Run blue = run("BLUE", "");
        System.out.print(red.report + blue.report);

        assertEquals(12, red.match.scored(), "RED pollen scored in AUTO (the starter route: 12)\n" + red.report);
        assertEquals(2, red.match.tips(), "RED tips\n" + red.report);
        for (double t : red.match.tipTimes()) {
            assertTrue(t + 4.0 <= 38.0, "tip at " + t + " s completes after TELEOP starts");
        }
        List<String> expected = new ArrayList<>();
        for (TwoTipStateMachineAuto.State s : TwoTipStateMachineAuto.State.values()) expected.add(s.name());
        assertEquals(expected, red.states, "every state ran, in order\n" + red.report);
        assertTrue(red.doneAt() < AUTO_PERIOD_S, "parked at " + red.doneAt() + " s\n" + red.report);
        assertTrue(BiobuzzMatch.inchesFromPark(red.rlog, true) < PARKED_WITHIN_IN,
                "RED ended " + BiobuzzMatch.inchesFromPark(red.rlog, true) + " in from the park spot\n" + red.report);
        assertTrue(BiobuzzMatch.inchesFromPark(blue.rlog, false) < PARKED_WITHIN_IN,
                "BLUE ended " + BiobuzzMatch.inchesFromPark(blue.rlog, false) + " in from the park spot\n" + blue.report);

        assertTrue(red.match.unintendedContacts().isEmpty(), "contact: " + red.match.unintendedContacts() + "\n" + red.report);
        // Zenith's starter route turns while it drives, and the 22 in robot slides square to the
        // south wall onto and off the garden lane. On the same route this follower strafes 32 % where
        // Zenith's runtime strafes 29 %: up to 35 % here.
        assertTrue(red.match.strafeFraction() < MAX_STRAFE_FRACTION, String.format(Locale.ROOT,
                "strafing is %.0f %% of the driving\n%s", 100 * red.match.strafeFraction(), red.report));

        java.util.Set<String> keys = RlogDecodedCompare.decodedKeys(red.rlog);
        for (String k : new String[] {"Auto/PlannedRoute", "Auto/State", "Field/Robot", "Field/Pollen"}) {
            assertTrue(keys.stream().anyMatch(x -> x.endsWith(k)), "RLOG has no " + k);
        }

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
        org.firstinspires.ftc.teamcode.aim.Alliance a = red ? org.firstinspires.ftc.teamcode.aim.Alliance.RED
                : org.firstinspires.ftc.teamcode.aim.Alliance.BLUE;
        Pose2d[] route = plannedRoute(a);
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
                    // For AdvantageScope: the whole planned route once (a trajectory), the state every tick.
                    if (time[0] == 0) Logger.recordOutput("Auto/PlannedRoute", Pose2d.struct, route);
                    time[0] += dt;
                    Object s = OpMode.blackboard.get(TwoTipStateMachineAuto.STATE_KEY);
                    if (s != null) Logger.recordOutput("Auto/State", (String) s);
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

    /**
     * The route the state machine drives, as AdvantageScope poses: each leg from the last one's
     * end, through AutoPoses' points (Bezier legs are cubic, the rest straight), 12 samples a leg.
     */
    static Pose2d[] plannedRoute(org.firstinspires.ftc.teamcode.aim.Alliance alliance) {
        com.pedropathing.math.Pose[][] legs = {
                {AutoPoses.START, AutoPoses.SCORE_SOUTH},
                bezier(AutoPoses.SCORE_SOUTH, AutoPoses.TO_OTHER_SIDE, AutoPoses.FLOWER_NORTH_APPROACH),
                {AutoPoses.FLOWER_NORTH_APPROACH, AutoPoses.FLOWER_NORTH_PICKUP},
                bezier(AutoPoses.FLOWER_NORTH_PICKUP, AutoPoses.TO_SCORE_NORTH, AutoPoses.SCORE_NORTH),
                bezier(AutoPoses.SCORE_NORTH, AutoPoses.TO_GARDEN, AutoPoses.ABOVE_GARDEN_LANE),
                {AutoPoses.ABOVE_GARDEN_LANE, AutoPoses.GARDEN_SWEEP_START},
                {AutoPoses.GARDEN_SWEEP_START, AutoPoses.GARDEN_SWEEP_END},
                {AutoPoses.GARDEN_SWEEP_END, AutoPoses.OFF_GARDEN_LANE},
                bezier(AutoPoses.OFF_GARDEN_LANE, AutoPoses.DRIVE_BACK, AutoPoses.SCORE_NORTH),
                bezier(AutoPoses.SCORE_NORTH, AutoPoses.TO_PARK_LANE, AutoPoses.PARK_LANE),
                {AutoPoses.PARK_LANE, AutoPoses.PARK}};
        List<Pose2d> out = new ArrayList<>();
        for (com.pedropathing.math.Pose[] leg : legs) {
            com.pedropathing.math.Pose[] p = new com.pedropathing.math.Pose[leg.length];
            for (int i = 0; i < leg.length; i++) p[i] = AutoPoses.of(alliance, leg[i]);
            double px = p[0].x(), py = p[0].y();
            for (int k = 1; k <= 12; k++) {
                double t = k / 12.0, x, y;
                // de Casteljau: a straight line for two points, a Bezier curve for more.
                double[] xs = new double[p.length], ys = new double[p.length];
                for (int i = 0; i < p.length; i++) {
                    xs[i] = p[i].x();
                    ys[i] = p[i].y();
                }
                for (int n = p.length - 1; n > 0; n--) {
                    for (int i = 0; i < n; i++) {
                        xs[i] += t * (xs[i + 1] - xs[i]);
                        ys[i] += t * (ys[i + 1] - ys[i]);
                    }
                }
                x = xs[0];
                y = ys[0];
                out.add(AdvantageScopeFrame.pose2d(x, y, Math.atan2(y - py, x - px)));
                px = x;
                py = y;
            }
        }
        return out.toArray(new Pose2d[0]);
    }

    /** start, the control points, end: one Bezier leg. */
    static com.pedropathing.math.Pose[] bezier(com.pedropathing.math.Pose start, com.pedropathing.math.Pose[] controls,
                                               com.pedropathing.math.Pose end) {
        com.pedropathing.math.Pose[] all = new com.pedropathing.math.Pose[controls.length + 2];
        all[0] = start;
        System.arraycopy(controls, 0, all, 1, controls.length);
        all[all.length - 1] = end;
        return all;
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
