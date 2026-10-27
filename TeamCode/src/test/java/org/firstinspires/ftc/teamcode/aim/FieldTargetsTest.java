package org.firstinspires.ftc.teamcode.aim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class FieldTargetsTest {

    @Test
    void theCellFollowsTheHalfOfTheFieldWithHysteresisAtTheLine() {
        assertEquals(FieldTargets.Cell.NORTH, FieldTargets.cellForHalf(30.0, FieldTargets.Cell.SOUTH));
        assertEquals(FieldTargets.Cell.SOUTH, FieldTargets.cellForHalf(-30.0, FieldTargets.Cell.NORTH));
        // Within 2 in of y = 0 the current target is kept, so it does not flicker on the line.
        assertEquals(FieldTargets.Cell.SOUTH, FieldTargets.cellForHalf(1.5, FieldTargets.Cell.SOUTH));
        assertEquals(FieldTargets.Cell.NORTH, FieldTargets.cellForHalf(-1.5, FieldTargets.Cell.NORTH));
        assertEquals(FieldTargets.Cell.NORTH, FieldTargets.cellForHalf(2.5, FieldTargets.Cell.SOUTH));
    }
}
