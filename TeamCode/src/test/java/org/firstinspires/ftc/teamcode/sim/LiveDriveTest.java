package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.firstinspires.ftc.teamcode.auto.AutoPoses;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.horizon36596.simloop.sim.live.GamepadInputSource;
import org.horizon36596.simloop.sim.live.GamepadInputTrack;
import org.horizon36596.simloop.sim.live.GamepadState;
import org.junit.jupiter.api.Test;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The live-drive plumbing (LiveDrive, GlfwGamepadSource's mapping) without a controller or a clock:
 * a scripted input stands in for the pad and the run is unpaced. What a live session adds on top is
 * only the controller poll, RealTimePacer's waiting and the window.
 */
class LiveDriveTest {

    private static final LiveDrive.Settings SHORT = new LiveDrive.Settings(Alliance.RED, 4, 3.0);
    private static final int INIT = SHORT.initTicks();

    @Test
    void theInputReachesTheTeleopAndTheRecordedTrackReplaysToTheSameLog() throws Exception {
        Path rlog = Paths.get("build", "sim", "live-test.rlog");
        Path trackPath = Paths.get("build", "sim", "live-test.gamepad.txt");
        // 1 s of full forward on gamepad1 just after PLAY, then centred.
        GamepadInputSource script = tick -> new GamepadInputTrack.Frame(tick,
                tick >= INIT + 5 && tick < INIT + 55 ? pad(Map.of("left_stick_y", -1f)) : GamepadState.AT_REST,
                GamepadState.AT_REST);

        LiveDrive.Result live;
        try (GamepadInputTrack.Writer track = new GamepadInputTrack.Writer(trackPath, SHORT.metadata(), true)) {
            live = LiveDrive.run(SHORT, script, track, ScenarioRunner.NO_PACING, rlog, null);
        }

        // RED starts facing +Y: forward on the stick is up the field.
        double movedY = live.sim.drivePlant.getY() - AutoPoses.START.y();
        assertTrue(movedY > 20.0, "1 s of full forward should move the robot well up the field, moved " + movedY);
        assertEquals(AutoPoses.START.x(), live.sim.drivePlant.getX(), 1.0, "forward must not drift sideways");
        assertEquals(-1, live.endedAtFrame, "nobody ended this one early");

        GamepadInputTrack track = GamepadInputTrack.read(trackPath);
        assertEquals(live.framesFed, track.frames().size(), "one recorded frame per loop the teleop was given");
        LiveDrive.Settings again = LiveDrive.Settings.fromMetadata(track.metadata());
        Path replayed = Paths.get("build", "sim", "live-test-replay.rlog");
        LiveDrive.run(again, track.replay(), null, ScenarioRunner.NO_PACING, replayed, null);
        List<RlogDecodedCompare.Mismatch> diff = RlogDecodedCompare.compare(rlog, replayed);
        assertTrue(diff.isEmpty(), "the replay should log what the session logged: " + RlogDecodedCompare.summarise(diff));
    }

    @Test
    void watchingARunDoesNotChangeIt() throws Exception {
        // The window reads the field every other loop; reading must not move anything.
        GamepadInputSource script = tick -> new GamepadInputTrack.Frame(tick,
                tick >= INIT + 5 && tick < INIT + 55 ? pad(Map.of("left_stick_y", -1f, "right_trigger", 1f))
                        : GamepadState.AT_REST, GamepadState.AT_REST);
        Path unwatched = Paths.get("build", "sim", "live-test-unwatched.rlog");
        Path watched = Paths.get("build", "sim", "live-test-watched.rlog");
        LiveDrive.run(SHORT, script, null, ScenarioRunner.NO_PACING, unwatched, null);
        int[] shown = {0};
        LiveDrive.run(SHORT, script, null, ScenarioRunner.NO_PACING, watched, new LiveDrive.View() {
            @Override public void show(LiveDrive.Snapshot snapshot) { shown[0]++; }
            @Override public boolean closed() { return false; }
        });
        assertTrue(shown[0] > 100, "the view should have been shown every other loop, was shown " + shown[0]);
        List<RlogDecodedCompare.Mismatch> diff = RlogDecodedCompare.compare(unwatched, watched);
        assertTrue(diff.isEmpty(), "a watched run should log what an unwatched one does: " + RlogDecodedCompare.summarise(diff));
    }

    @Test
    void holdingBackAndStartEndsTheSessionAndTheRestIsAtRest() throws Exception {
        int endHold = (int) Math.round(LiveDrive.END_HOLD_SECONDS / LiveDrive.TICK_S);
        LiveDrive.Result ended = LiveDrive.run(SHORT, forwardHolding(true), null, ScenarioRunner.NO_PACING,
                Paths.get("build", "sim", "live-test-end.rlog"), null);
        LiveDrive.Result notEnded = LiveDrive.run(SHORT, forwardHolding(false), null, ScenarioRunner.NO_PACING,
                Paths.get("build", "sim", "live-test-noend.rlog"), null);

        assertEquals(INIT + 5 + endHold - 1, ended.endedAtFrame, "ends once the chord has been held for the hold time");
        assertEquals(-1, notEnded.endedAtFrame, "BACK alone does not end it");
        // Both push the stick forward for the whole period; the ended one stops pushing after ~1 s of 3.
        double endedY = ended.sim.drivePlant.getY() - AutoPoses.START.y();
        double notEndedY = notEnded.sim.drivePlant.getY() - AutoPoses.START.y();
        assertTrue(endedY > 10.0, "it drove until the chord ended it, moved " + endedY);
        assertTrue(endedY < 0.75 * notEndedY, String.format(
                "after the end the pad is at rest: moved %.1f in, %.1f in without ending", endedY, notEndedY));
    }

    /** Full forward all period; from just after PLAY, BACK held, and START too if {@code chord}. */
    private static GamepadInputSource forwardHolding(boolean chord) {
        return tick -> {
            Set<String> down = new HashSet<>();
            if (tick >= INIT + 5) {
                down.add("back");
                if (chord) down.add("start");
            }
            return new GamepadInputTrack.Frame(tick, GamepadState.of(Map.of("left_stick_y", -1f), down),
                    GamepadState.AT_REST);
        };
    }

    @Test
    void glfwLayoutMapsOntoTheFtcGamepad() {
        float[] axes = {0.03f, -1f, 0.5f, 0f, -1f, 1f}; // LX in the deadzone, LY full up, RX half, LT rest, RT full
        boolean[] buttons = new boolean[15];
        buttons[org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_A] = true;
        buttons[org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_DPAD_LEFT] = true;
        buttons[org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER] = true;
        GamepadState s = GlfwGamepadSource.toGamepadState(axes, buttons);

        assertEquals(0f, s.axis("left_stick_x"), 1e-6, "inside the deadzone is centred");
        assertEquals(-1f, s.axis("left_stick_y"), 1e-6, "stick up is negative y, as on the Driver Station");
        assertEquals(0.5f, s.axis("right_stick_x"), 1e-6);
        assertEquals(0f, s.axis("left_trigger"), 1e-6, "GLFW's -1 is a released trigger");
        assertEquals(1f, s.axis("right_trigger"), 1e-6, "GLFW's +1 is a full trigger");
        assertEquals(Set.of("a", "dpad_left", "right_bumper"), s.buttonsDown());
    }

    private static GamepadState pad(Map<String, Float> axes) {
        return GamepadState.of(axes, Collections.emptySet());
    }
}
