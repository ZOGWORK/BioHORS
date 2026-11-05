package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.sim.desktop.FieldWindow;
import org.horizon36596.simloop.opmode.OpModePhase;

import java.util.ArrayList;
import java.util.List;

/**
 * A live session's window (simdesktop's FieldWindow, which holds the drawing: this source set
 * cannot see java.awt). The robot is the plant's TRUE pose; the text is the period clock, the
 * field's counts and the teleop's last Driver Station telemetry.
 */
final class LiveView implements LiveDrive.View {

    private final FieldWindow window;

    private LiveView(FieldWindow window) {
        this.window = window;
    }

    static LiveView open(String title) throws Exception {
        return new LiveView(FieldWindow.open(title));
    }

    @Override
    public void show(LiveDrive.Snapshot s) {
        double[] pollen = new double[2 * s.pollenIn.size()];
        for (int i = 0; i < s.pollenIn.size(); i++) {
            pollen[2 * i] = s.pollenIn.get(i)[0];
            pollen[2 * i + 1] = s.pollenIn.get(i)[1];
        }
        List<String> lines = new ArrayList<>();
        lines.add(String.format("Pose x=%.1f y=%.1f h=%.0f deg (true)", s.xIn, s.yIn, Math.toDegrees(s.headingRad)));
        lines.add(String.format("Held %d  Scored %d  Tips %d", s.held, s.scored, s.tips));
        lines.add("");
        lines.add("Driver Station telemetry:");
        lines.addAll(s.telemetry);
        window.show(status(s), s.xIn, s.yIn, s.headingRad, pollen, lines);
    }

    @Override
    public boolean closed() {
        return window.isClosed();
    }

    void close() {
        window.close();
    }

    private static String status(LiveDrive.Snapshot s) {
        if (s.ended) return "ENDED: finishing the log...";
        if (s.phase == OpModePhase.INIT || s.phase == OpModePhase.INIT_LOOP) {
            return String.format("INIT: PLAY in %.1f s", Math.max(0, LiveDrive.INIT_SECONDS - s.simSeconds));
        }
        return String.format("%s  %.1f s", s.phase, Math.max(0, s.simSeconds - LiveDrive.INIT_SECONDS));
    }
}
