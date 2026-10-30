package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The teleop's loop-time readout (MainHorsTeleOp.LoopTimeStats), fed unequal loop times. */
class LoopTimeStatsTest {

    @Test
    void emptyUntilTheFirstLoopIsAdded() {
        MainHorsTeleOp.LoopTimeStats stats = new MainHorsTeleOp.LoopTimeStats(50);
        assertTrue(stats.isEmpty());
        stats.add(12.0);
        assertFalse(stats.isEmpty());
    }

    @Test
    void lastAverageAndPeakAreThreeDifferentNumbers() {
        MainHorsTeleOp.LoopTimeStats stats = new MainHorsTeleOp.LoopTimeStats(50);
        stats.add(10.0);
        stats.add(40.0);
        stats.add(16.0);
        assertEquals(16.0, stats.lastMs(), 1e-9, "last");
        assertEquals(22.0, stats.averageMs(), 1e-9, "average of the three so far");
        assertEquals(40.0, stats.peakMs(), 1e-9, "peak");
    }

    @Test
    void anOldSlowLoopLeavesTheAverageButStaysThePeak() {
        MainHorsTeleOp.LoopTimeStats stats = new MainHorsTeleOp.LoopTimeStats(50);
        stats.add(100.0);
        for (int i = 0; i < 49; i++) stats.add(10.0);
        assertEquals((100.0 + 49 * 10.0) / 50, stats.averageMs(), 1e-9, "the slow loop is still in the window");

        stats.add(20.0); // the 51st loop pushes the slow one out
        assertEquals((49 * 10.0 + 20.0) / 50, stats.averageMs(), 1e-9, "the slow loop has left the average");
        assertEquals(100.0, stats.peakMs(), 1e-9, "but it is still the peak");
        assertEquals(20.0, stats.lastMs(), 1e-9, "last");
    }
}
