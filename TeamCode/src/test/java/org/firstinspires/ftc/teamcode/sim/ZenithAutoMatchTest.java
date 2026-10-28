package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.simloop.simclock.SimClockLoader;
import org.horizon36596.zenith.AutoSource;
import org.horizon36596.zenith.Json;
import org.horizon36596.zenith.solverslib.AutoSpec;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The deployed Zenith auto, run as a BIOBUZZ match in SimLoop.
 *
 * The real robot code runs: the auto file from the APK's assets, Zenith's runtime and SolversLib
 * scheduler, BioHorsZenithRobot and its commands, Pedro's follower with its mecanum drivetrain and
 * Pinpoint localizer. Only the hardware (BioHorsSim) and the field (BiobuzzMatch) are simulated.
 *
 * The OpMode is stopped at 30 s the way the field stops it, so every score counted here is an AUTO
 * score. RED runs twice and must log identically; BLUE (the point mirror) must score the same.
 *
 * zenith.json's sim.command runs this with -Dzenith.auto=<auto>, and it writes
 * TeamCode/build/sim/<auto>.trace.json for `zenith sim` to read.
 */
class ZenithAutoMatchTest {

    static final double AUTO_PERIOD_S = 30.0;
    static final double TICK_S = OpModeRunner.DEFAULT_TICK_SECONDS;
    /** INIT to PLAY: long enough for the Pinpoint to calibrate. */
    static final int INIT_LOOP_TICKS = 25;
    static final String AUTO_NAME = "two-tip";

    /** The starter route: 4 preloads tip the south cell (3 nectar staged in it, season tip table), 4 flower + 4 garden pollen tip the north cell: 12 pollen, 2 tips. */
    static final int EXPECTED_SCORED = 12;
    static final double PARKED_WITHIN_IN = 2.0;   // SET BY HAND: the follower's end tolerance, with margin
    static final int EXPECTED_TIPS = 2;
    static final double MAX_STRAFE_FRACTION = 0.30;

    private final File root = projectRoot();
    private final File assets = new File(root, "TeamCode/src/main/assets/autos");

    @AfterEach
    void clearBlackboard() {
        OpMode.blackboard.clear();
    }

    @Test
    void theTwoTipAutoScoresSixteenAndTipsTwiceAsRedAndAsBlue() throws IOException {
        String auto = System.getProperty("zenith.auto", AUTO_NAME);
        Result red1 = runMatch(auto, "RED", "run1");
        Result red2 = runMatch(auto, "RED", "");
        Result blue = runMatch(auto, "BLUE", "");

        System.out.print(red2.report + blue.report);

        // ---- RED: the score, counted only as AUTO (the OpMode stops at 30 s) ----
        assertEquals(EXPECTED_SCORED, red2.match.scored(), "RED pollen scored in AUTO\n" + red2.report);
        assertEquals(EXPECTED_TIPS, red2.match.tips(), "RED tips in AUTO\n" + red2.report);
        for (double t : red2.match.tipTimes()) {
            // A tip counts as AUTO if its swing (4 s, DSim APPROX) completes by 38 s (manual 10.5.B).
            assertTrue(t + 4.0 <= 38.0, "tip at " + t + " s completes after TELEOP starts");
        }

        // ---- every step ran, in order, none interrupted, the last ended before the buzzer ----
        assertEquals(red2.planned, red2.recorded, "the steps that ran\n" + red2.report);
        assertTrue(red2.lastEndS < AUTO_PERIOD_S, "the last step ended at " + red2.lastEndS + " s\n" + red2.report);
        assertTrue(red2.interrupted.isEmpty(), "interrupted steps: " + red2.interrupted + "\n" + red2.report);
        assertTrue(BiobuzzMatch.inchesFromPark(red2.rlog, true) < PARKED_WITHIN_IN,
                "RED ended " + BiobuzzMatch.inchesFromPark(red2.rlog, true) + " in from the park spot\n" + red2.report);

        // ---- the run is sound: no unintended contact, little strafing, belief agrees with truth ----
        assertTrue(red2.match.unintendedContacts().isEmpty(),
                "structure contact: " + red2.match.unintendedContacts() + "\n" + red2.report);
        // Zenith's starter route turns while it drives (its own legs warn at 22 %), and the 22 in robot
        // has to slide square to the south wall onto and off the garden lane: up to 30 %.
        assertTrue(red2.match.strafeFraction() < MAX_STRAFE_FRACTION, String.format(Locale.ROOT,
                "strafing is %.0f %% of the driving\n%s", 100 * red2.match.strafeFraction(), red2.report));
        assertTrue(red2.maxBeliefErrorIn < 2.0, String.format(Locale.ROOT,
                "belief vs truth up to %.2f in\n%s", red2.maxBeliefErrorIn, red2.report));

        // ---- AdvantageScope: the robot, its belief and every pollen are in the log as poses ----
        java.util.Set<String> keys = RlogDecodedCompare.decodedKeys(red2.rlog);
        for (String k : new String[] {"Field/Robot", "Field/RobotBelief", "Field/Pollen", "Field/HiveModel",
                "Auto/PlannedRoute", "Auto/DrivenPath"}) {
            assertTrue(keys.stream().anyMatch(x -> x.endsWith(k)), "RLOG has no " + k + "; keys: " + keys);
        }
        System.out.println("AdvantageScope keys: " + keys.stream().filter(x -> x.contains("Field/")).sorted()
                .collect(java.util.stream.Collectors.toList()));
        java.util.List<java.util.Map<String, String>> frames = RlogDecodedCompare.decode(red2.rlog);
        for (int i : new int[] {30, 700, frames.size() - 1}) {
            java.util.Map<String, String> f = frames.get(Math.min(i, frames.size() - 1));
            for (java.util.Map.Entry<String, String> e : f.entrySet()) {
                if (e.getKey().contains("Field/Robot")) {
                    System.out.println("frame " + i + " " + e.getKey() + " = " + e.getValue());
                }
                if (e.getKey().endsWith("Field/Pollen")) {
                    String v = e.getValue();
                    System.out.println("frame " + i + " " + e.getKey() + " = " + v.substring(0, Math.min(240, v.length())));
                }
            }
        }

        // ---- deterministic: two RED runs decode to the same log ----
        List<RlogDecodedCompare.Mismatch> diff = RlogDecodedCompare.compare(red1.rlog, red2.rlog);
        assertTrue(diff.isEmpty(), "two identical RED runs logged differently: " + RlogDecodedCompare.summarise(diff));

        // ---- BLUE, the mirror, scores the same ----
        assertEquals(red2.match.scored(), blue.match.scored(), "BLUE should score what RED scores\n" + blue.report);
        assertEquals(red2.match.tips(), blue.match.tips(), "BLUE should tip what RED tips\n" + blue.report);
        assertTrue(blue.match.unintendedContacts().isEmpty(), "BLUE structure contact: " + blue.match.unintendedContacts());
        assertEquals(blue.planned, blue.recorded, "BLUE steps that ran");
        assertTrue(BiobuzzMatch.inchesFromPark(blue.rlog, false) < PARKED_WITHIN_IN,
                "BLUE ended " + BiobuzzMatch.inchesFromPark(blue.rlog, false) + " in from the park spot\n" + blue.report);
    }

    // ------------------------------------------------------------------ physics

    /**
     * WHAT-IF shot for the physics run: a steep, grippy exit (70 deg, 0.8 of the rim speed). Not
     * measured. It is the kind of shot under which the team's 1600 rpm test target reaches the cell
     * (ShotSweepTest prints the table). The CAD-read 44 deg cannot score from the auto's spots at any
     * rpm. Film one real shot and put the measured angle and speed in ShotModel.
     */
    static final double WHAT_IF_PITCH_DEG = 70.0;
    static final double WHAT_IF_GRIP = 0.8;

    @Test
    void theAutoWithRealBallsInPhysics() throws IOException {
        double savedPitch = ShotModel.POLLEN_PITCH_DEG, savedGrip = ShotModel.GRIP_EFFICIENCY;
        ShotModel.POLLEN_PITCH_DEG = WHAT_IF_PITCH_DEG;
        ShotModel.GRIP_EFFICIENCY = WHAT_IF_GRIP;
        try {
            String auto = System.getProperty("zenith.auto", AUTO_NAME);
            Result a = runMatch(auto, "RED", "physics1", true);
            Result b = runMatch(auto, "RED", "physics", true);
            System.out.print("PHYSICS (what-if shot " + WHAT_IF_PITCH_DEG + " deg, grip " + WHAT_IF_GRIP + ")"
                    + b.report);

            // Every pollen the robot held left it, and each launch is accounted for: seated or missed.
            assertEquals(EXPECTED_SCORED, b.match.launched(), "pollen launched\n" + b.report);
            assertEquals(b.match.launched(), b.match.scored() + b.match.missed(),
                    "a launch neither seated nor missed\n" + b.report);
            for (org.horizon36596.simloop.field.GamePiece p : b.match.shotPieces()) {
                assertTrue(Math.abs(p.getFieldXInches()) <= 70.674 && Math.abs(p.getFieldYInches()) <= 70.674,
                        "a ball left the field: " + p);
            }
            // The hive and the balls are in the log for AdvantageScope's 3D field.
            java.util.Set<String> keys = RlogDecodedCompare.decodedKeys(b.rlog);
            for (String k : new String[] {"Field/Hive", "Field/Pollen", "Match/hiveAngleDeg"}) {
                assertTrue(keys.stream().anyMatch(x -> x.endsWith(k)), "RLOG has no " + k);
            }
            List<RlogDecodedCompare.Mismatch> diff = RlogDecodedCompare.compare(a.rlog, b.rlog);
            assertTrue(diff.isEmpty(), "two physics runs logged differently: " + RlogDecodedCompare.summarise(diff));
        } finally {
            ShotModel.POLLEN_PITCH_DEG = savedPitch;
            ShotModel.GRIP_EFFICIENCY = savedGrip;
        }
    }

    // ------------------------------------------------------------------ one match

    static final class Result {
        BiobuzzMatch match;
        Path rlog;
        List<String> planned = new ArrayList<>();
        List<String> recorded = new ArrayList<>();
        List<String> interrupted = new ArrayList<>();
        double lastEndS;
        double maxBeliefErrorIn;
        String report;
    }

    private Result runMatch(String auto, String alliance, String suffix) throws IOException {
        return runMatch(auto, alliance, suffix, false);
    }

    private Result runMatch(String auto, String alliance, String suffix, boolean physics) throws IOException {
        Json robotJson = readJson(new File(assets, "robot.json"));
        Json fieldJson = readJson(new File(assets, "biobuzz.field.json"));
        Json autoJson = readJson(new File(assets, auto + ".auto.json"));
        Json holds = autoJson.get("start").opt("holds");
        int preload = holds == null ? 0 : (int) holds.optNum("pollen", 0);
        AutoSpec spec = AutoSpec.load(AutoSource.directory(assets), auto);

        BioHorsSim sim = new BioHorsSim();
        boolean red = "RED".equals(alliance);
        BiobuzzMatch match = new BiobuzzMatch(sim, fieldJson, robotJson, red, preload);
        if (physics) match.usePhysics();
        double sx = spec.startPose.getX(), sy = spec.startPose.getY(), sh = spec.startPose.getHeading();
        if (spec.mirrorsInto(alliance)) {
            sx = -sx;
            sy = -sy;
            sh = sh + Math.PI;
        }
        sim.placeAt(sx, sy, sh);

        String tag = auto + "-" + alliance.toLowerCase(Locale.ROOT) + (suffix.isEmpty() ? "" : "-" + suffix);
        File traceFile = new File(root, "TeamCode/build/sim/"
                + (red && suffix.isEmpty() ? auto : tag) + ".trace.json");
        Path rlog = new File(root, "TeamCode/build/sim/" + tag + ".rlog").toPath();
        Files.createDirectories(rlog.getParent());
        Files.deleteIfExists(traceFile.toPath());

        OpMode.blackboard.clear();
        OpMode.blackboard.put("biohors.alliance", alliance); // what Choose Alliance would have set
        OpMode.blackboard.put(TracedZenithAuto.AUTO, auto);
        OpMode.blackboard.put(TracedZenithAuto.SOURCE_DIR, assets.getPath());
        OpMode.blackboard.put(TracedZenithAuto.TRACE_FILE, traceFile.getPath());
        OpMode.blackboard.put(TracedZenithAuto.TICK_S, TICK_S);
        OpMode.blackboard.put(TracedZenithAuto.DRIVE_PLANT, sim.drivePlant);

        // For AdvantageScope: the planned route once, as a trajectory. Both autos drive the same route
        // (AutoPosesTest keeps AutoPoses equal to this draft).
        org.psilynx.psikit.core.wpi.math.Pose2d[] route = StateMachineAutoMatchTest.plannedRoute(
                red ? org.firstinspires.ftc.teamcode.aim.Alliance.RED : org.firstinspires.ftc.teamcode.aim.Alliance.BLUE);
        boolean[] routeLogged = {false};
        OpModeRunResult result = OpModeRunner.forTeamClass(TracedZenithAuto.class, sim.hardwareMap)
                // Robot code, SolversLib, Pedro and Zenith's runtime all read the sim clock; Zenith's
                // runtime has to share SolversLib's loader because its classes extend SolversLib's.
                .teamPackages("org.firstinspires.ftc.teamcode.", "com.seattlesolvers.solverslib.",
                        SimClockLoader.PEDRO_PATHING, "org.horizon36596.zenith.")
                .keepBlackboard(true)
                .tickSeconds(TICK_S)
                .initLoopTicks(INIT_LOOP_TICKS)
                // The field stops the OpMode at 30 s: exactly the AUTO period, no margin.
                .activeTicks((int) Math.round(AUTO_PERIOD_S / TICK_S))
                .eachTick(dt -> {
                    sim.step(dt);
                    match.step(dt);
                    if (!routeLogged[0]) {
                        org.psilynx.psikit.core.Logger.recordOutput("Auto/PlannedRoute",
                                org.psilynx.psikit.core.wpi.math.Pose2d.struct, route);
                        routeLogged[0] = true;
                    }
                })
                .runLogged(BioHorsSim.ROBOT_CONFIG, rlog);

        Result r = new Result();
        r.match = match;
        r.rlog = rlog;
        for (AutoSpec.Step s : spec.steps) r.planned.add(s.id);
        // Members of a race end when the race does, so the losing member is interrupted by design.
        List<String> raceMembers = new ArrayList<>();
        collectRaceMembers(autoJson.get("steps"), false, raceMembers);

        assertTrue(traceFile.isFile(), "the run wrote no trace at " + traceFile);
        Json trace = readJson(traceFile);
        StringBuilder rep = new StringBuilder();
        rep.append(String.format(Locale.ROOT, "%n== %s %s: %d ticks%n", auto, alliance, result.ticks()));
        for (Json step : trace.get("steps").items()) {
            // The trace also lists the members of parallel groups; the plan is the top-level steps.
            if (r.planned.contains(step.str("id"))) r.recorded.add(step.str("id"));
            if (step.get("interrupted").bool() && !raceMembers.contains(step.str("id"))) {
                r.interrupted.add(step.str("id"));
            }
            r.lastEndS = Math.max(r.lastEndS, step.num("endS"));
            rep.append(String.format(Locale.ROOT, "  %-22s %6.2f -> %6.2f s%s%n", step.str("id"),
                    step.num("startS"), step.num("endS"), step.get("interrupted").bool() ? "  INTERRUPTED" : ""));
        }
        r.maxBeliefErrorIn = maxBeliefError(trace);
        rep.append(String.format(Locale.ROOT,
                "  scored %d (missed %d, launched %d, collected %d), tips %d at %s, held at end %d%n",
                match.scored(), match.missed(), match.launched(), match.collected(), match.tips(), match.tipTimes(),
                match.held()));
        rep.append(String.format(Locale.ROOT, "  strafe %.0f %% of %.0f in driven; belief vs truth max %.2f in%n",
                100 * match.strafeFraction(), match.drivenIn(), r.maxBeliefErrorIn));
        rep.append("  contacts: ").append(match.contacts()).append('\n');
        rep.append("  misses: ").append(match.missLog()).append('\n');
        for (String w : result.warnings()) rep.append("  SimLoop warning: ").append(w).append('\n');
        r.report = rep.toString();
        return r;
    }

    /** Ids of the steps directly inside a race or deadline group (they may end interrupted). */
    private static void collectRaceMembers(Json steps, boolean inRace, List<String> out) {
        for (Json s : steps.items()) {
            if (inRace) out.add(s.str("id"));
            Json children = s.opt("steps");
            if (children != null) {
                String mode = s.optText("mode", "");
                collectRaceMembers(children, mode.equals("race") || mode.equals("deadline"), out);
            }
        }
    }

    /** Largest distance between the robot's believed pose and the plant's true pose, inches. */
    private static double maxBeliefError(Json trace) {
        Json poses = trace.opt("poses");
        Json truth = trace.opt("truthPoses");
        if (poses == null || truth == null) return Double.NaN;
        double worst = 0;
        int n = Math.min(poses.size(), truth.size());
        for (int i = 0; i < n; i++) {
            Json a = poses.at(i), b = truth.at(i);
            double d = Math.hypot(a.at(1).num() - b.at(1).num(), a.at(2).num() - b.at(2).num());
            worst = Math.max(worst, d);
        }
        return worst;
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

    @SuppressWarnings("unused")
    private static String read(File f) throws IOException {
        return new String(Files.readAllBytes(f.toPath()), StandardCharsets.UTF_8);
    }
}
