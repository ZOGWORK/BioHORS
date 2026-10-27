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
        final boolean[] bothSpunBeforeTogether = {false};
        final boolean[] togetherStarted = {false};
        final double[] maxPower = new double[2];
        OpModeRunResult result = OpModeRunner.forTeamClass(TUNER, sim.hardwareMap)
                .initLoopTicks(5)
                .activeTicks(9000) // 180 s: far more than the tuner needs; it ends by itself
                .eachTick(dt -> {
                    sim.step(dt);
                    double p = Math.abs(sim.pollenShooter.getPower());
                    double n = Math.abs(sim.nectarShooter.getPower());
                    // Both spinning is allowed only in the final together check, which starts
                    // once both have been swept to full power alone.
                    if (p > 0.01 && n > 0.01) {
                        if (maxPower[0] > 0.9 && maxPower[1] > 0.9) togetherStarted[0] = true;
                        else bothSpunBeforeTogether[0] = true;
                    }
                    maxPower[0] = Math.max(maxPower[0], p);
                    maxPower[1] = Math.max(maxPower[1], n);
                })
                .run();

        List<String> log = result.telemetry().loggedLines();
        String all = String.join("\n", log) + "\n" + String.join("\n", result.telemetry().lastTransmission());
        System.out.println(all);

        assertFalse(bothSpunBeforeTogether[0], "the tuner must tune one shooter at a time");
        assertTrue(togetherStarted[0], "then it must run both together for the together check");
        assertTrue(all.contains("Together check (both on)") && all.contains(": PASS"), "together check should pass");
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

    /**
     * A battery that sags with both shooters' draw (BioHorsSim.enableShooterBatterySag). The
     * per-shooter tuning and the together check must still pass, and the together run must be
     * at a lower voltage than either shooter alone, which is the case the check exists for.
     */
    @Test
    void underASaggingBatteryBothShootersStillHoldTogether() {
        BioHorsSim sim = new BioHorsSim();
        sim.enableShooterBatterySag();
        OpModeRunResult result = OpModeRunner.forTeamClass(TUNER, sim.hardwareMap)
                .initLoopTicks(5)
                .activeTicks(9000)
                .eachTick(sim::step)
                .run();
        String all = String.join("\n", result.telemetry().loggedLines());
        System.out.println(all);
        Matcher m = Pattern.compile("Pollen [0-9]+ rpm within ([0-9.]+) %.*Nectar [0-9]+ rpm within ([0-9.]+) %.*"
                + "battery ([0-9.]+) V together vs ([0-9.]+) / ([0-9.]+) V alone; band [0-9]+ %: (PASS|FAIL)").matcher(all);
        assertTrue(m.find(), "no together check in: " + all);
        double together = Double.parseDouble(m.group(3));
        double alonePollen = Double.parseDouble(m.group(4));
        double aloneNectar = Double.parseDouble(m.group(5));
        assertTrue(together < Math.min(alonePollen, aloneNectar) - 0.1,
                "together should sag the battery more than either alone: " + m.group(0));
        assertEquals("PASS", m.group(6), "voltage-compensated feedforward should hold both: " + m.group(0));
        double maxTps = number(all + "\n" + String.join("\n", result.telemetry().lastTransmission()),
                "POLLEN_SHOOTER[.]ffReferenceMaxTicksPerSec = ([0-9.]+);");
        assertEquals(2800.0, maxTps, 2800.0 * 0.03, "the sweep's voltage normalisation should still find 2800 at 12 V");
    }

    /**
     * Tuned on a fresh battery, checked together on a tired one: the battery drains from 12.9 V
     * over the run (about 11.4 V by the together check), plus the shooters' own sag. This is the
     * case voltage-compensated feedforward exists for; without it the shooters fall short.
     */
    @Test
    void tunedOnAFreshBatteryTheyStillHoldTogetherOnATiredOne() {
        BioHorsSim sim = new BioHorsSim();
        sim.enableShooterBatterySag(12.9, 0.0125);
        OpModeRunResult result = OpModeRunner.forTeamClass(TUNER, sim.hardwareMap)
                .initLoopTicks(5)
                .activeTicks(9000)
                .eachTick(sim::step)
                .run();
        String all = String.join("\n", result.telemetry().loggedLines());
        System.out.println(all);
        Matcher m = Pattern.compile("Pollen [0-9]+ rpm within ([0-9.]+) %.*Nectar [0-9]+ rpm within ([0-9.]+) %.*"
                + "battery ([0-9.]+) V together vs ([0-9.]+) / ([0-9.]+) V alone; band [0-9]+ %: (PASS|FAIL)").matcher(all);
        assertTrue(m.find(), "no together check in: " + all);
        assertTrue(Double.parseDouble(m.group(3)) < 11.8, "the together check should run on a tired battery: " + m.group(0));
        assertEquals("PASS", m.group(6), "voltage-compensated feedforward should hold both: " + m.group(0));
    }

    /**
     * A pollen flywheel whose speed wobbles (a slipping belt, say) never gives a steady sweep
     * point. The tuner must report it FAILED with nothing to paste, still tune nectar, and skip
     * the together check.
     */
    @Test
    void aShooterThatNeverSettlesIsReportedFailedWithNothingToPaste() {
        BioHorsSim sim = new BioHorsSim();
        final double[] t = {0};
        OpModeRunResult result = OpModeRunner.forTeamClass(TUNER, sim.hardwareMap)
                .initLoopTicks(5)
                .activeTicks(9000)
                .eachTick(dt -> {
                    t[0] += dt;
                    sim.pollenShooter.setExternalTorqueFraction(0.08 * Math.sin(2 * Math.PI * t[0] / 0.5));
                    sim.step(dt);
                })
                .run();
        String all = String.join("\n", result.telemetry().loggedLines()) + "\n"
                + String.join("\n", result.telemetry().lastTransmission());
        System.out.println(all);
        assertTrue(all.contains("Pollen: FAILED"), "pollen should fail: " + all);
        assertFalse(all.contains("POLLEN_SHOOTER.kP"), "a failed shooter must have nothing to paste");
        assertTrue(all.contains("NECTAR_SHOOTER.kP"), "nectar should still be tuned");
        assertTrue(all.contains("Together check SKIPPED"), "no together check with a failed shooter");
        assertTrue(all.contains("DONE WITH FAILURES"), "the final screen must not just say DONE");
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
