package org.firstinspires.ftc.teamcode.sim;

import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_LEFT_TRIGGER;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_LEFT_X;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_LEFT_Y;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_RIGHT_X;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_RIGHT_Y;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_A;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_B;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_BACK;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_DPAD_DOWN;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_DPAD_LEFT;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_DPAD_RIGHT;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_DPAD_UP;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_GUIDE;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_LEFT_BUMPER;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_LEFT_THUMB;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_RIGHT_THUMB;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_START;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_X;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_Y;
import static org.lwjgl.glfw.GLFW.GLFW_JOYSTICK_1;
import static org.lwjgl.glfw.GLFW.GLFW_JOYSTICK_LAST;
import static org.lwjgl.glfw.GLFW.GLFW_PRESS;

import org.horizon36596.simloop.sim.live.GamepadInputSource;
import org.horizon36596.simloop.sim.live.GamepadInputTrack;
import org.horizon36596.simloop.sim.live.GamepadState;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWGamepadState;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * A real USB game controller (Xbox, Logitech F310 in X mode, PS4/PS5, ...) as SimLoop gamepad input,
 * read through GLFW's gamepad API (LWJGL, test-only). The first controller GLFW sees is gamepad1, the
 * second, if one is plugged in, gamepad2.
 *
 * GLFW maps every known controller onto one standard layout (SDL's controller database), and that
 * layout already matches the FTC Gamepad: stick up is NEGATIVE y, as on the Driver Station. Only the
 * triggers differ: GLFW reports them in [-1, 1] with -1 at rest, the Gamepad wants [0, 1].
 *
 * GLFW is started on the first {@link #next} call, from the thread that polls it, and polled once
 * per sim loop. Nothing here reads the wall clock.
 */
final class GlfwGamepadSource implements GamepadInputSource {

    /** A resting stick reads a little off centre; below this it is centred. SET BY HAND. */
    static final float STICK_DEADZONE = 0.06f;

    private final List<Integer> joysticks = new ArrayList<>();
    private GLFWGamepadState state;
    private boolean started;

    @Override
    public GamepadInputTrack.Frame next(int tick) {
        if (!started) start();
        GLFW.glfwPollEvents();
        GamepadState g1 = read(0);
        GamepadState g2 = read(1);
        return new GamepadInputTrack.Frame(tick, g1, g2);
    }

    /** The controllers in use, as GLFW names them, gamepad1 first. Empty until the first poll. */
    List<String> names() {
        List<String> out = new ArrayList<>();
        for (int jid : joysticks) out.add(GLFW.glfwGetGamepadName(jid));
        return out;
    }

    private void start() {
        started = true;
        if (!GLFW.glfwInit()) {
            throw new IllegalStateException("GLFW did not start, so no controller can be read on this machine");
        }
        state = GLFWGamepadState.create();
        List<String> notGamepads = new ArrayList<>();
        for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; jid++) {
            if (!GLFW.glfwJoystickPresent(jid)) continue;
            if (GLFW.glfwJoystickIsGamepad(jid)) {
                if (joysticks.size() < 2) joysticks.add(jid);
            } else {
                notGamepads.add(GLFW.glfwGetJoystickName(jid) + " (GUID " + GLFW.glfwGetJoystickGUID(jid) + ")");
            }
        }
        if (joysticks.isEmpty()) {
            throw new IllegalStateException("No game controller found. Plug one in over USB (or pair it) BEFORE "
                    + "starting the session, and check Windows sees it (Settings > Bluetooth & devices > "
                    + "Game controllers)."
                    + (notGamepads.isEmpty() ? "" : " Found joysticks GLFW has no gamepad layout for: "
                    + String.join(", ", notGamepads) + ". A Logitech F310 must be switched to X (XInput) mode."));
        }
    }

    /** Controller n's state now; at rest if it is not there or was unplugged mid-session. */
    private GamepadState read(int n) {
        if (n >= joysticks.size()) return GamepadState.AT_REST;
        if (!GLFW.glfwGetGamepadState(joysticks.get(n), state)) return GamepadState.AT_REST;
        float[] axes = new float[6];
        for (int i = 0; i < axes.length; i++) axes[i] = state.axes(i);
        boolean[] buttons = new boolean[15];
        for (int i = 0; i < buttons.length; i++) buttons[i] = state.buttons(i) == GLFW_PRESS;
        return toGamepadState(axes, buttons);
    }

    /**
     * GLFW's standard gamepad layout to an FTC gamepad: axes indexed by GLFW_GAMEPAD_AXIS_*, buttons
     * by GLFW_GAMEPAD_BUTTON_*.
     */
    static GamepadState toGamepadState(float[] axes, boolean[] buttons) {
        Map<String, Float> a = new HashMap<>();
        a.put("left_stick_x", stick(axes[GLFW_GAMEPAD_AXIS_LEFT_X]));
        a.put("left_stick_y", stick(axes[GLFW_GAMEPAD_AXIS_LEFT_Y]));
        a.put("right_stick_x", stick(axes[GLFW_GAMEPAD_AXIS_RIGHT_X]));
        a.put("right_stick_y", stick(axes[GLFW_GAMEPAD_AXIS_RIGHT_Y]));
        a.put("left_trigger", trigger(axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]));
        a.put("right_trigger", trigger(axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER]));

        Set<String> down = new HashSet<>();
        press(down, buttons, GLFW_GAMEPAD_BUTTON_A, "a");                 // cross on a PlayStation pad
        press(down, buttons, GLFW_GAMEPAD_BUTTON_B, "b");                 // circle
        press(down, buttons, GLFW_GAMEPAD_BUTTON_X, "x");                 // square
        press(down, buttons, GLFW_GAMEPAD_BUTTON_Y, "y");                 // triangle
        press(down, buttons, GLFW_GAMEPAD_BUTTON_LEFT_BUMPER, "left_bumper");
        press(down, buttons, GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER, "right_bumper");
        press(down, buttons, GLFW_GAMEPAD_BUTTON_BACK, "back");           // share / view
        press(down, buttons, GLFW_GAMEPAD_BUTTON_START, "start");         // options / menu
        press(down, buttons, GLFW_GAMEPAD_BUTTON_GUIDE, "guide");
        press(down, buttons, GLFW_GAMEPAD_BUTTON_LEFT_THUMB, "left_stick_button");
        press(down, buttons, GLFW_GAMEPAD_BUTTON_RIGHT_THUMB, "right_stick_button");
        press(down, buttons, GLFW_GAMEPAD_BUTTON_DPAD_UP, "dpad_up");
        press(down, buttons, GLFW_GAMEPAD_BUTTON_DPAD_RIGHT, "dpad_right");
        press(down, buttons, GLFW_GAMEPAD_BUTTON_DPAD_DOWN, "dpad_down");
        press(down, buttons, GLFW_GAMEPAD_BUTTON_DPAD_LEFT, "dpad_left");
        return GamepadState.of(a, down);
    }

    private static float stick(float v) {
        float c = Math.max(-1f, Math.min(1f, v));
        return Math.abs(c) < STICK_DEADZONE ? 0f : c;
    }

    private static float trigger(float v) {
        return Math.max(0f, Math.min(1f, (v + 1f) / 2f));
    }

    private static void press(Set<String> down, boolean[] buttons, int index, String name) {
        if (buttons[index]) down.add(name);
    }
}
