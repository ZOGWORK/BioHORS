package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The Shooter Tuner OpMode, run unchanged against BioHorsSim's shooters.
 *
 * The sim flywheel's truth is known (BioHorsSim: 2800 ticks/s free speed at full power, no
 * friction, a steady 12 V battery), so the tuner must find it, pick a P that holds the target, and
 * never spin both shooters at once.
 */
class ShooterTunerSimTest {

    static final String TUNER = "org.firstinspires.ftc.teamcode.tuning.ShooterTuner";

    @Test
    void theTunerMeasuresBothShootersOneAtATimeAndItsNumbersHoldTarget() {
        BioHorsSim sim = new BioHorsSim();
        final boolean[] bothSpun = {false};
        final double[] maxPower = new double[2];
        OpModeRunResult result = OpModeRunner.forTeamClass(TUNER, sim.hardwareMap)
                .initLoopTicks(5)
                .activeTicks(9000) // 180 s: far more than the tuner needs; it ends by itself
                .eachTick(dt -> {
                    sim.step(dt);
                    double p = Math.abs(sim.pollenShooter.getPower());
                    double n = Math.abs(sim.nectarShooter.getPower());
                    if (p > 0.01 && n > 0.01) bothSpun[0] = true;
                    maxPower[0] = Math.max(maxPower[0], p);
                    maxPower[1] = Math.max(maxPower[1], n);
                })
                .run();

        List<String> log = result.telemetry().loggedLines();
        String all = String.join("\n", log) + "\n" + String.join("\n", result.telemetry().lastTransmission());
        System.out.println(all);

        assertFalse(bothSpun[0], "the tuner must run one shooter at a time");
        assertTrue(maxPower[0] > 0.9 && maxPower[1] > 0.9, "both shooters should have been swept to full power");

        for (String shooter : new String[] {"POLLEN_SHOOTER", "NECTAR_SHOOTER"}) {
            double maxTps = number(all, shooter + "[.]ffReferenceMaxTicksPerSec = ([0-9.]+);");
            double kS = number(all, shooter + "[.]kS = ([0-9.]+);");
            double kP = number(all, shooter + "[.]kP = ([0-9.]+);");
            assertEquals(2800.0, maxTps, 2800.0 * 0.015, shooter + " feedforward should find the 2800 ticks/s free speed");
            assertTrue(kS < 0.01, shooter + " kS should be ~0 on a frictionless flywheel, got " + kS);
            assertTrue(kP > 0, shooter + " should find a P");
            assertTrue(all.contains(shooter + ".kI = 0.0;") && all.contains(shooter + ".kD = 0.0;"), "I and D stay 0");
        }
        for (String label : new String[] {"Pollen", "Nectar"}) {
            Matcher m = Pattern.compile(label + " hold check at [0-9]+ rpm: spin-up ([0-9.]+) s, steady within "
                    + "([0-9.]+) %, dip to [0-9]+ rpm recovered in ([0-9.]+) s").matcher(all);
            assertTrue(m.find(), "no hold check for " + label);
            double spinUp = Double.parseDouble(m.group(1));
            double band = Double.parseDouble(m.group(2));
            double recovery = Double.parseDouble(m.group(3));
            assertTrue(spinUp < 2.5, label + " spin-up " + spinUp + " s");
            assertTrue(band <= 2.0, label + " steady band " + band + " %");
            assertTrue(recovery < 1.5, label + " dip recovery " + recovery + " s");
        }
    }

    private static double number(String text, String regex) {
        Matcher m = Pattern.compile(regex).matcher(text);
        assertTrue(m.find(), "missing in the tuner output: " + regex);
        return Double.parseDouble(m.group(1));
    }

    @SuppressWarnings("unused")
    private static String fmt(double d) {
        return String.format(Locale.ROOT, "%.3f", d);
    }
}
