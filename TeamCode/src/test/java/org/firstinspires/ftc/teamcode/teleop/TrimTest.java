package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** The teleop's shooter trim on top of the distance table (MainHorsTeleOp.trimmed). */
class TrimTest {

    @Test
    void aPressMovesTheTargetOneStep() {
        assertEquals(50.0, MainHorsTeleOp.trimmed(0.0, 50.0, 1600.0), 1e-9);
        assertEquals(-50.0, MainHorsTeleOp.trimmed(0.0, -50.0, 1600.0), 1e-9);
    }

    @Test
    void trimDownStopsAtAZeroTarget() {
        assertEquals(-1600.0, MainHorsTeleOp.trimmed(-1580.0, -50.0, 1600.0), 1e-9);
    }

    @Test
    void afterTheTableDropsBelowTheTrimOneTrimUpPressRaisesTheTargetByOneStep() {
        // Trimmed to zero at a table rpm of 1600, then the table drops to 1500: target still 0.
        double trim = MainHorsTeleOp.trimmed(-1600.0, 50.0, 1500.0);
        assertEquals(50.0, Math.max(0.0, 1500.0 + trim), 1e-9, "one press, one step up from zero");
    }
}
