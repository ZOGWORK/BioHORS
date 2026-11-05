package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.firstinspires.ftc.teamcode.auto.AutoPoses;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.opmode.RecordingTelemetry;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.horizon36596.simloop.sim.SimElapsedTime;
import org.horizon36596.simloop.sim.live.GamepadInputSource;
import org.horizon36596.simloop.sim.live.GamepadInputTrack;
import org.horizon36596.simloop.sim.live.GamepadState;
import org.horizon36596.simloop.sim.live.RealTimePacer;
import org.horizon36596.simloop.sim.live.RecordingGamepadInputSource;
import org.horizon36596.simloop.simclock.SimClockLoader;
import org.horizon36596.zenith.Json;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * MainHorsTeleOp, unchanged, on BioHorsSim and the BIOBUZZ field (BiobuzzMatch, physics mode), with
 * its gamepads fed from a {@link GamepadInputSource} each loop: a real controller (LiveDriveSession),
 * a recorded track (replay), or a script (LiveDriveTest).
 *
 * Every loop, in this order: the input's next frame goes onto gamepad1 / gamepad2 (before the
 * teleop's loop reads them), the teleop runs, the devices and plants update, the field steps, the
 * view is told, and the pacer waits for the wall clock to catch up. The pacer only waits, so a paced
 * run and an unpaced replay of its input track compute the same thing and log the same RLOG.
 *
 * Ending early: hold BACK + START for {@link #END_HOLD_SECONDS}, or close the view window. The rest
 * of the period then runs with both gamepads at rest and no pacing, so the RLOG is complete. The
 * input track records those at-rest frames too, so a replay ends the same way.
 *
 * The live part (the pacer, the controller, the window) is in LiveDriveSession; nothing here reads
 * the wall clock.
 */
final class LiveDrive {

    static final String TELEOP = MainHorsTeleOpSimTest.TELEOP;
    static final double TICK_S = OpModeRunner.DEFAULT_TICK_SECONDS;
    /** INIT before PLAY: time to get your hands on the pad (and the Pinpoint calibrates). SET BY HAND. */
    static final double INIT_SECONDS = 3.0;
    /** FTC driver-controlled period, 2:00. SPEC (game manual). */
    static final double TELEOP_SECONDS = 120.0;
    /** Pollen aboard at the start: the auto's four preloads (autos/two-tip.auto.json start.holds). */
    static final int DEFAULT_PRELOAD = 4;
    /** BACK + START held this long ends the session early. SET BY HAND. */
    static final double END_HOLD_SECONDS = 1.0;
    /** The view is redrawn every this many loops (25 Hz at 50 Hz loops). SET BY HAND. */
    static final int VIEW_EVERY_TICKS = 2;

    /** What a session is: the same three things make a replay match its recording. */
    static final class Settings {
        final Alliance alliance;
        final int preload;
        final double seconds;

        Settings(Alliance alliance, int preload, double seconds) {
            this.alliance = alliance;
            this.preload = preload;
            this.seconds = seconds;
        }

        int initTicks() {
            return (int) Math.round(INIT_SECONDS / TICK_S);
        }

        int activeTicks() {
            return (int) Math.round(seconds / TICK_S);
        }

        /** Written into the input track's header, so a replay can rebuild the session. */
        Map<String, String> metadata() {
            Map<String, String> m = new LinkedHashMap<>();
            m.put("opMode", TELEOP);
            m.put("tickSeconds", Double.toString(TICK_S));
            m.put("alliance", alliance.name());
            m.put("preload", Integer.toString(preload));
            m.put("seconds", Double.toString(seconds));
            return m;
        }

        static Settings fromMetadata(Map<String, String> m) {
            if (!TELEOP.equals(m.get("opMode")) || !Double.toString(TICK_S).equals(m.get("tickSeconds"))) {
                throw new IllegalArgumentException("not a live-drive track of " + TELEOP + " at " + TICK_S
                        + " s a loop: " + m);
            }
            return new Settings(Alliance.valueOf(m.get("alliance")), Integer.parseInt(m.get("preload")),
                    Double.parseDouble(m.get("seconds")));
        }
    }

    /** What the view draws, one loop's worth, copied so the drawing thread never touches the sim. */
    static final class Snapshot {
        final OpModePhase phase;
        final double simSeconds;
        final double xIn, yIn, headingRad;
        final List<double[]> pollenIn;
        final List<String> telemetry;
        final int held, scored, tips;
        final boolean ended;

        Snapshot(OpModePhase phase, double simSeconds, double xIn, double yIn, double headingRad,
                 List<double[]> pollenIn, List<String> telemetry, int held, int scored, int tips, boolean ended) {
            this.phase = phase;
            this.simSeconds = simSeconds;
            this.xIn = xIn;
            this.yIn = yIn;
            this.headingRad = headingRad;
            this.pollenIn = pollenIn;
            this.telemetry = telemetry;
            this.held = held;
            this.scored = scored;
            this.tips = tips;
            this.ended = ended;
        }
    }

    /** Where a session shows itself while it runs. */
    interface View {
        void show(Snapshot snapshot);

        /** The driver closed it: end the session early. */
        boolean closed();
    }

    static final class Result {
        final OpModeRunResult run;
        final BiobuzzMatch match;
        final BioHorsSim sim;
        /** Frames taken from the input, one a loop (INIT, PLAY and STOP included). */
        final int framesFed;
        /** The frame at which the driver ended early, or -1 if the period ran out. */
        final int endedAtFrame;

        Result(OpModeRunResult run, BiobuzzMatch match, BioHorsSim sim, int framesFed, int endedAtFrame) {
            this.run = run;
            this.match = match;
            this.sim = sim;
            this.framesFed = framesFed;
            this.endedAtFrame = endedAtFrame;
        }
    }

    private LiveDrive() {
    }

    /**
     * Runs one session and writes its RLOG to {@code rlog}. {@code track}, if not null, records every
     * frame the teleop was given; {@code pacer} is RealTimePacer for a live session or
     * {@code ScenarioRunner.NO_PACING}; {@code view} may be null.
     */
    static Result run(Settings settings, GamepadInputSource input, GamepadInputTrack.Writer track,
                      ScenarioRunner.TickPacer pacer, Path rlog, View view) throws IOException {
        File assets = new File(projectRoot(), "TeamCode/src/main/assets/autos");
        boolean red = settings.alliance == Alliance.RED;
        BioHorsSim sim = new BioHorsSim();
        BiobuzzMatch match = new BiobuzzMatch(sim, readJson(new File(assets, "biobuzz.field.json")),
                readJson(new File(assets, "robot.json")), red, settings.preload).usePhysics();

        // Start where the auto starts, and tell the teleop so, as an auto that just ran would.
        Pose start = AutoPoses.of(settings.alliance, AutoPoses.START);
        sim.placeAt(start.x(), start.y(), start.heading());
        OpMode.blackboard.clear();
        OpMode.blackboard.put("biohors.alliance", settings.alliance.name());
        OpMode.blackboard.put("biohors.pose", new double[] {start.x(), start.y(), start.heading()});
        OpMode.blackboard.put("biohors.poseTimer", new SimElapsedTime(new FakeTimer())); // saved just now

        EndableInput endable = new EndableInput(input, view);
        GamepadInputSource fed = track == null ? endable : new RecordingGamepadInputSource(endable, track);
        FakeTimer timer = new FakeTimer();
        final OpModePhase[] phaseNow = {OpModePhase.INIT};
        final int[] frame = {0};
        final int[] tick = {0};
        Files.createDirectories(rlog.toAbsolutePath().getParent());

        OpModeRunner runner = OpModeRunner.forTeamClass(TELEOP, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .keepBlackboard(true)
                .timer(timer)
                .initLoopTicks(settings.initTicks())
                .activeTicks(settings.activeTicks());
        try {
            runner.gamepads((phase, t, g1, g2) -> {
                        phaseNow[0] = phase;
                        GamepadInputTrack.Frame f = fed.next(frame[0]++);
                        (f == null ? GamepadState.AT_REST : f.gamepad1).applyTo(g1);
                        (f == null ? GamepadState.AT_REST : f.gamepad2).applyTo(g2);
                    })
                    .eachTick(dt -> {
                        sim.step(dt);
                        match.step(dt);
                        if (view != null && tick[0] % VIEW_EVERY_TICKS == 0) {
                            view.show(snapshot(phaseNow[0], timer.time(), sim, match, runner, endable.ended()));
                        }
                        // Once the driver has ended it, the rest runs flat out to finish the log.
                        if (!endable.ended()) pacer.afterTick(tick[0], timer.time());
                        tick[0]++;
                    });
            OpModeRunResult result = runner.runLogged(BioHorsSim.ROBOT_CONFIG, rlog);
            if (view != null) view.show(snapshot(OpModePhase.STOP, timer.time(), sim, match, runner, true));
            return new Result(result, match, sim, frame[0], endable.endedAtFrame);
        } finally {
            OpMode.blackboard.clear();
        }
    }

    /** The input, until the driver ends the session; at rest after that. */
    private static final class EndableInput implements GamepadInputSource {
        private final GamepadInputSource input;
        private final View view;
        private int heldFrames;
        private volatile int endedAtFrame = -1;

        EndableInput(GamepadInputSource input, View view) {
            this.input = input;
            this.view = view;
        }

        boolean ended() {
            return endedAtFrame >= 0;
        }

        @Override
        public GamepadInputTrack.Frame next(int frame) {
            if (!ended()) {
                GamepadInputTrack.Frame f = input.next(frame);
                boolean endChord = f != null && f.gamepad1.isDown("back") && f.gamepad1.isDown("start");
                heldFrames = endChord ? heldFrames + 1 : 0;
                if (heldFrames >= Math.round(END_HOLD_SECONDS / TICK_S) || (view != null && view.closed())) {
                    endedAtFrame = frame;
                } else if (f != null) {
                    return f;
                }
            }
            return new GamepadInputTrack.Frame(frame, GamepadState.AT_REST, GamepadState.AT_REST);
        }
    }

    private static Snapshot snapshot(OpModePhase phase, double simSeconds, BioHorsSim sim, BiobuzzMatch match,
                                     OpModeRunner runner, boolean ended) {
        List<double[]> pollen = new ArrayList<>();
        for (GamePiece p : match.fieldPieces()) pollen.add(new double[] {p.getFieldXInches(), p.getFieldYInches()});
        List<String> lines = Collections.emptyList();
        Telemetry t = runner.opMode().telemetry;
        if (t instanceof RecordingTelemetry) lines = new ArrayList<>(((RecordingTelemetry) t).lastTransmission());
        return new Snapshot(phase, simSeconds, sim.drivePlant.getX(), sim.drivePlant.getY(),
                sim.drivePlant.getHeading(), pollen, lines, match.held(), match.scored(), match.tips(), ended);
    }

    /** How often the live pacer gave up catching up (the sim ran slower than real time). */
    static int lagResets(ScenarioRunner.TickPacer pacer) {
        return pacer instanceof RealTimePacer ? ((RealTimePacer) pacer).lagResets() : 0;
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
