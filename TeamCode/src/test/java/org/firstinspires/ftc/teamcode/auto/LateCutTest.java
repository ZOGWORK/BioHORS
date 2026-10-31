package org.firstinspires.ftc.teamcode.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.firstinspires.ftc.teamcode.auto.TwoTipStateMachineAuto.State;
import org.junit.jupiter.api.Test;

/**
 * The state machine's late cutoffs: off the garden with what is aboard, and the last volley skipped
 * for the park only when it is reached late. On time (the planned route's timings) nothing is cut.
 */
class LateCutTest {

    static final double LEAVE = TwoTipStateMachineAuto.LEAVE_GARDEN_BY_S;
    static final double SKIP = TwoTipStateMachineAuto.SKIP_LAST_VOLLEY_AFTER_S;

    @Test
    void onThePlannedTimingsNothingIsCut() {
        // From the sim's planned run: sweep 15.7-16.5 s, last volley 20.7-22.8 s.
        assertNull(TwoTipStateMachineAuto.lateCut(State.SWEEP_GARDEN, 16.0, 15.7));
        assertNull(TwoTipStateMachineAuto.lateCut(State.WAIT_GARDEN_FULL, 16.6, 16.5));
        assertNull(TwoTipStateMachineAuto.lateCut(State.SHOOT_GARDEN, 22.0, 20.7));
    }

    @Test
    void stillAtTheGardenLateLeavesIt() {
        for (State s : new State[] {State.SLIDE_ONTO_LANE, State.SWEEP_GARDEN, State.WAIT_GARDEN_FULL}) {
            assertNull(TwoTipStateMachineAuto.lateCut(s, LEAVE - 0.02, 15.0), s + " just before the cutoff");
            assertEquals(State.OFF_GARDEN_LANE, TwoTipStateMachineAuto.lateCut(s, LEAVE, 15.0), s.name());
        }
    }

    @Test
    void theLastVolleyIsSkippedOnlyWhenReachedLate() {
        assertEquals(State.TO_PARK_LANE, TwoTipStateMachineAuto.lateCut(State.SHOOT_GARDEN, SKIP, SKIP));
        assertNull(TwoTipStateMachineAuto.lateCut(State.SHOOT_GARDEN, SKIP + 1.0, SKIP - 0.5),
                "a volley already under way is let finish");
    }

    @Test
    void otherStatesAreNeverCut() {
        // Their onward paths are not safe from just anywhere (round the hive), so lateness waits.
        for (State s : new State[] {State.TO_GARDEN, State.DRIVE_BACK, State.TO_OTHER_SIDE, State.COLLECT_FLOWER}) {
            assertNull(TwoTipStateMachineAuto.lateCut(s, 29.0, 28.0), s.name());
        }
    }
}
