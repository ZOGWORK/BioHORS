package org.firstinspires.ftc.teamcode.tuning.autotune;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

/** FtcAutoTune's ticks/s gains, converted, must command the same power ShooterController does. */
class AutoTuneGainsTest {

    static final double TICKS_PER_REV = 28.0; // goBILDA 6000 RPM, 1:1 (SPEC)

    /** ShooterController's feedforward + P, written out from its update(). */
    static double shooterPower(AutoTuneGains.Converted g, double targetRpm, double errRpm, double volts) {
        double maxTicks = (volts / AutoTuneGains.V_REF) * g.ffReferenceMaxTicksPerSec;
        double targetTicks = targetRpm * TICKS_PER_REV / 60.0;
        return targetTicks / maxTicks * 1.0 + g.kP * errRpm;
    }

    @Test
    void matchesAutoTunePowerAtTheSweepVoltage() {
        double kP = 0.0004, kF = 1.0 / 2400.0, vSweep = 12.8;
        AutoTuneGains.Converted g = AutoTuneGains.convert(kP, 0, 0, kF, TICKS_PER_REV, vSweep);

        double targetRpm = 1600, errRpm = 50;
        double targetTicks = targetRpm * TICKS_PER_REV / 60.0;
        double errTicks = errRpm * TICKS_PER_REV / 60.0;
        double autoTunePower = kP * errTicks + kF * targetTicks;

        assertEquals(autoTunePower, shooterPower(g, targetRpm, errRpm, vSweep), 1e-12);
    }

    @Test
    void feedforwardRisesAsTheBatteryDrops() {
        AutoTuneGains.Converted g = AutoTuneGains.convert(0, 0, 0, 1.0 / 2400.0, TICKS_PER_REV, 12.8);
        double atSweep = shooterPower(g, 1600, 0, 12.8);
        assertEquals(atSweep * 12.8 / 11.5, shooterPower(g, 1600, 0, 11.5), 1e-12);
    }

    @Test
    void scalesPidTermsByTicksPerRpm() {
        AutoTuneGains.Converted g = AutoTuneGains.convert(0.0006, 0.0002, 0.00003, 1.0 / 2400.0, TICKS_PER_REV, 12.0);
        assertEquals(0.0006 * 28.0 / 60.0, g.kP, 1e-15);
        assertEquals(0.0002 * 28.0 / 60.0, g.kI, 1e-15);
        assertEquals(0.00003 * 28.0 / 60.0, g.kD, 1e-15);
        assertEquals(2400.0, g.ffReferenceMaxTicksPerSec, 1e-9);
    }

    @Test
    void rejectsAMissingSweep() {
        assertThrows(IllegalArgumentException.class,
                () -> AutoTuneGains.convert(0.0004, 0, 0, Double.NaN, TICKS_PER_REV, 12.0));
        assertThrows(IllegalArgumentException.class,
                () -> AutoTuneGains.convert(0.0004, 0, 0, 1.0 / 2400.0, TICKS_PER_REV, 0));
    }
}
