package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

/** Each cue fires exactly once, at its time; a loop that jumps past both gives only the later one. */
class EndgameAlertsTest {

    @Test
    void theEndgameAndTheFinalWarningFireOnceEachAtTheirTimes() {
        EndgameAlerts alerts = new EndgameAlerts();
        List<String> fired = new ArrayList<>();
        for (int i = 0; i <= 130 * 50; i++) {
            double t = i * 0.02;
            EndgameAlerts.Cue c = alerts.update(t);
            if (c != EndgameAlerts.Cue.NONE) fired.add(c + "@" + Math.round(t * 100) / 100.0);
        }
        assertEquals(List.of("ENDGAME@" + (EndgameAlerts.TELEOP_S - EndgameAlerts.ENDGAME_S),
                "FINAL_WARNING@" + (EndgameAlerts.TELEOP_S - EndgameAlerts.FINAL_WARNING_S)), fired);
    }

    @Test
    void jumpingPastBothGivesOnlyTheFinalWarning() {
        EndgameAlerts alerts = new EndgameAlerts();
        assertEquals(EndgameAlerts.Cue.NONE, alerts.update(1.0));
        assertEquals(EndgameAlerts.Cue.FINAL_WARNING, alerts.update(115.0));
        assertEquals(EndgameAlerts.Cue.NONE, alerts.update(116.0));
    }

    @Test
    void noTimeNoCue() {
        assertEquals(EndgameAlerts.Cue.NONE, new EndgameAlerts().update(Double.NaN));
    }
}
