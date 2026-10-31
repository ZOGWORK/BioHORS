package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * A pass/fail list for INIT, before PLAY: what the drive team can still fix on the field. Each line
 * starts "OK" or "!!". Shown by the teleop and both autos.
 *
 * Checks: the Pinpoint is READY; the battery is at MIN_START_V or more; each shooter's encoder reads
 * (about) zero with the wheel at rest (a noisy or spinning reading is a wiring or motor fault). A
 * dead encoder also reads zero at rest, so this cannot catch that one: the encoder check while
 * shooting does (ShooterController).
 */
public final class PreMatchCheck {

    private PreMatchCheck() {
    }

    /** A fresh match battery should rest at least this high, volts. SET BY HAND: a charged 12 V
     *  FTC battery rests near 12.8-13 V; below 12.5 V it is part used. */
    public static double MIN_START_V = 12.5;
    /** At rest the shooter encoders should read under this, rpm. SET BY HAND. */
    public static double REST_RPM = 50.0;

    /** One line per check, each "OK ..." or "!! ...". */
    public static List<String> lines(BioHorsRobot robot) {
        List<String> out = new ArrayList<>();
        out.add(robot.pinpointReady() ? "OK Pinpoint READY" : "!! Pinpoint NOT READY (wait, or check its cable)");
        double v = robot.batteryVolts();
        if (!Double.isFinite(v)) {
            out.add("!! battery: no voltage reading");
        } else {
            out.add(String.format(Locale.ROOT, v >= MIN_START_V ? "OK battery %.1f V" : "!! battery %.1f V (swap it: under %.1f V)",
                    v, MIN_START_V));
        }
        double p = robot.pollenShooter.measuredRpmNow();
        double n = robot.nectarShooter.measuredRpmNow();
        boolean still = Math.abs(p) < REST_RPM && Math.abs(n) < REST_RPM;
        out.add(String.format(Locale.ROOT, still ? "OK shooter encoders at rest" : "!! shooter encoders not at rest: pollen %.0f, nectar %.0f rpm",
                p, n));
        return out;
    }

    /** True when every line passed. */
    public static boolean allOk(List<String> lines) {
        for (String l : lines) if (!l.startsWith("OK")) return false;
        return true;
    }
}
