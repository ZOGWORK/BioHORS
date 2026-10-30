package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The Loop line's numbers (MainHorsTeleOp.LoopTimes): last, rolling average and peak. */
class LoopTimesTest {

    @Test
    void nothingRecordedHasNoSample() {
        MainHorsTeleOp.LoopTimes t = new MainHorsTeleOp.LoopTimes(50);
        assertFalse(t.hasSample());
        assertTrue(Double.isNaN(t.averageMs()));
    }

    @Test
    void beforeTheWindowFillsTheAverageIsOverWhatWasRecorded() {
        MainHorsTeleOp.LoopTimes t = new MainHorsTeleOp.LoopTimes(50);
        t.record(10.0);
        t.record(30.0);
        assertEquals(30.0, t.lastMs(), 1e-9);
        assertEquals(20.0, t.averageMs(), 1e-9, "over the two recorded, not padded with zeros");
        assertEquals(30.0, t.peakMs(), 1e-9);
    }

    @Test
    void aSlowLoopLeavesTheAverageAfterAWindowOfNewerLoopsButStaysTheMax() {
        MainHorsTeleOp.LoopTimes t = new MainHorsTeleOp.LoopTimes(50);
        t.record(100.0);                         // one slow loop
        for (int i = 0; i < 49; i++) t.record(20.0);
        assertEquals((100.0 + 49 * 20.0) / 50.0, t.averageMs(), 1e-9, "still inside the window");

        t.record(10.0);                          // the 51st: pushes the slow loop out
        assertEquals((49 * 20.0 + 10.0) / 50.0, t.averageMs(), 1e-9,
                "the average is the last 50 only: not since PLAY, not the last loop");
        assertEquals(10.0, t.lastMs(), 1e-9);
        assertEquals(100.0, t.peakMs(), 1e-9, "the peak is since PLAY");
    }
}
