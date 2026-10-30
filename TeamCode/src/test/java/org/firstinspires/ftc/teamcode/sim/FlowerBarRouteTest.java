package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * The team's rule for the flower bar in the Zenith autos (the deployed RED and BLUE files): it goes
 * down for the flower and stays down all the way back to the shooting spot. Nothing folds it up
 * before the volley (the score command lifts it to SHOOTING); it is stowed right after.
 * StateMachineAutoMatchTest checks the same on the hand-written auto, tick by tick.
 */
class FlowerBarRouteTest {

    @Test
    void theBarStaysDownFromTheFlowerToTheVolley() throws IOException {
        File assets = new File(StateMachineAutoMatchTest.projectRoot(), "TeamCode/src/main/assets/autos");
        for (String auto : new String[] {"two-tip", "two-tip-blue"}) {
            List<String> steps = new ArrayList<>();
            for (Json s : StateMachineAutoMatchTest.readJson(new File(assets, auto + ".auto.json")).get("steps").items()) {
                steps.add(s.str("kind").equals("command") ? s.str("name") : s.str("id"));
            }
            int down = steps.indexOf("flowerBarDown");
            assertTrue(down >= 0, auto + ": no flowerBarDown in " + steps);
            int volley = steps.subList(down, steps.size()).indexOf("score") + down;
            assertTrue(volley > down, auto + ": no volley after the flower in " + steps);
            assertTrue(!steps.subList(down, volley).contains("flowerBarStow"),
                    auto + ": the bar folds before the shooting spot: " + steps.subList(down, volley + 1));
            assertEquals("flowerBarStow", steps.get(volley + 1), auto + ": the step after the flower volley");
        }
    }
}
