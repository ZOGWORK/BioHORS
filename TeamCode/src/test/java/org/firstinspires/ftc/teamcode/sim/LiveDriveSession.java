package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.horizon36596.simloop.sim.live.GamepadInputTrack;
import org.horizon36596.simloop.sim.live.RealTimePacer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;

/**
 * Drive MainHorsTeleOp in the sim, in real time, with a USB game controller, and record it.
 * docs/LIVE_DRIVE.md. Not a test: it runs only when asked for, with
 *
 * <pre>.\gradlew.bat driveSim</pre>
 *
 * which runs this and nothing else (TeamCode/build.gradle). Options, each a -P on that line, which
 * the build passes in as sim.live.* properties: alliance (RED or BLUE, default RED), seconds (default
 * 120, the teleop period), preload (pollen aboard at the start, default 4), window (false: no window).
 *
 * Each session writes, under TeamCode/build/sim/live/, a timestamped RLOG for AdvantageScope and the
 * controller input track that made it. {@code -Preplay=<track>} (or {@code latest}) replays a track instead of reading
 * a controller: unpaced (or, with {@code -Pwatch=true}, in real time in the window), into a -replay
 * RLOG, and checks it decodes the same as the session's.
 *
 * The wall clock is read here, and only by RealTimePacer (to wait) and for the file names.
 */
@EnabledIfSystemProperty(named = "sim.live", matches = "true")
class LiveDriveSession {

    static final Path DIR = Paths.get("build", "sim", "live");

    @Test
    void drive() throws Exception {
        String replay = System.getProperty("sim.live.replay");
        if (replay != null && !replay.isEmpty()) {
            replay("latest".equals(replay) ? latestTrack() : Paths.get(replay).toAbsolutePath());
            return;
        }

        LiveDrive.Settings settings = new LiveDrive.Settings(
                Alliance.valueOf(System.getProperty("sim.live.alliance", "RED").toUpperCase()),
                Integer.parseInt(System.getProperty("sim.live.preload", Integer.toString(LiveDrive.DEFAULT_PRELOAD))),
                Double.parseDouble(System.getProperty("sim.live.seconds", Double.toString(LiveDrive.TELEOP_SECONDS))));
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"));
        Path rlog = DIR.resolve("drive-" + stamp + ".rlog").toAbsolutePath();
        Path trackPath = DIR.resolve("drive-" + stamp + ".gamepad.txt").toAbsolutePath();
        Files.createDirectories(rlog.getParent());

        GlfwGamepadSource controller = new GlfwGamepadSource();
        controller.next(0); // fails here, before any window, if no controller is plugged in
        System.out.println("[live] controllers: " + controller.names() + " (first = gamepad1)");
        System.out.println("[live] " + settings.alliance + ", " + settings.preload + " pollen aboard, "
                + settings.seconds + " s after " + LiveDrive.INIT_SECONDS + " s of INIT. Hold BACK + START to end.");

        LiveView view = Boolean.parseBoolean(System.getProperty("sim.live.window", "true"))
                ? LiveView.open("BioHORS live sim: " + settings.alliance) : null;
        RealTimePacer pacer = new RealTimePacer();
        LiveDrive.Result result;
        try (GamepadInputTrack.Writer track = new GamepadInputTrack.Writer(trackPath, settings.metadata(), false)) {
            result = LiveDrive.run(settings, controller, track, pacer, rlog, view);
        } finally {
            if (view != null) view.close();
        }

        System.out.println("[live] done: " + result.match.scored() + " scored, " + result.match.tips() + " tips"
                + (result.endedAtFrame >= 0 ? ", ended early by the driver" : "")
                + (LiveDrive.lagResets(pacer) > 0 ? "; the sim fell behind real time "
                + LiveDrive.lagResets(pacer) + " time(s)" : ""));
        System.out.println("[live] RLOG (drag into AdvantageScope): " + rlog);
        System.out.println("[live] input track (replay with driveSim -Preplay=<it>): " + trackPath);
    }

    /** The newest session's input track (file names sort by their timestamp). */
    private static Path latestTrack() throws java.io.IOException {
        Path newest = null;
        if (Files.isDirectory(DIR)) {
            try (java.util.stream.Stream<Path> files = Files.list(DIR)) {
                newest = files.filter(f -> f.getFileName().toString().endsWith(".gamepad.txt"))
                        .max(java.util.Comparator.comparing(f -> f.getFileName().toString())).orElse(null);
            }
        }
        if (newest == null) throw new IllegalStateException("no recorded session in " + DIR.toAbsolutePath());
        return newest.toAbsolutePath();
    }

    private static void replay(Path trackPath) throws Exception {
        GamepadInputTrack track = GamepadInputTrack.read(trackPath);
        LiveDrive.Settings settings = LiveDrive.Settings.fromMetadata(track.metadata());
        String name = trackPath.getFileName().toString().replace(".gamepad.txt", "");
        Path recorded = trackPath.resolveSibling(name + ".rlog");
        Path rlog = trackPath.resolveSibling(name + "-replay.rlog");
        // -Pwatch=true plays it back in real time in the window; otherwise it runs flat out, unseen.
        boolean watch = Boolean.getBoolean("sim.live.watch");
        LiveView view = watch ? LiveView.open("BioHORS replay: " + name) : null;
        RealTimePacer pacer = new RealTimePacer();
        try {
            LiveDrive.run(settings, track.replay(), null, watch ? pacer : ScenarioRunner.NO_PACING, rlog, view);
        } finally {
            if (view != null) view.close();
        }
        System.out.println("[live] replay RLOG: " + rlog
                + (LiveDrive.lagResets(pacer) > 0 ? " (fell behind real time " + LiveDrive.lagResets(pacer) + " time(s))" : ""));
        if (Files.isRegularFile(recorded)) {
            List<RlogDecodedCompare.Mismatch> diff = RlogDecodedCompare.compare(recorded, rlog);
            assertTrue(diff.isEmpty(), "the replay should log what the session logged: "
                    + RlogDecodedCompare.summarise(diff));
            System.out.println("[live] the replay logs the same as " + recorded);
        }
    }
}
