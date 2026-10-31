package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The warning follows a sustained low battery, not a spin-up sag, and clears when it recovers. */
class LowBatteryWarningTest {

    static final double TICK = 0.02;

    static void run(LowBatteryWarning w, double volts, double seconds) {
        for (double t = 0; t < seconds - 1e-9; t += TICK) w.update(volts, TICK);
    }

    @Test
    void aBriefSagDoesNotWarn() {
        LowBatteryWarning w = new LowBatteryWarning();
        run(w, 12.6, 1.0);
        run(w, 10.9, LowBatteryWarning.HOLD_S - 0.1);
        run(w, 12.4, 0.5);
        assertFalse(w.low());
    }

    @Test
    void aSustainedLowWarnsAndARecoveryClears() {
        LowBatteryWarning w = new LowBatteryWarning();
        run(w, 11.2, LowBatteryWarning.HOLD_S + TICK);
        assertTrue(w.low());
        run(w, 12.0, LowBatteryWarning.HOLD_S - 0.1);
        assertTrue(w.low(), "not cleared by a moment's rest");
        run(w, 12.0, 0.2);
        assertFalse(w.low());
    }

    @Test
    void noSensorNeverWarns() {
        LowBatteryWarning w = new LowBatteryWarning();
        run(w, Double.NaN, 5.0);
        assertFalse(w.low());
    }
}
