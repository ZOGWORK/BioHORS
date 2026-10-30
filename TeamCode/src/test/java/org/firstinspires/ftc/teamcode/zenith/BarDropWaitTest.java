package org.firstinspires.ftc.teamcode.zenith;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

/**
 * The Zenith autos drop the flower bar, wait MechanismConstants.FLOWER_BAR_DROP_S, then drive in:
 * the step order and the wait must match the state machine's (both alliances' files).
 */
class BarDropWaitTest {

    @Test
    void theBarDropsAndSettlesBeforeTheDriveIn() throws IOException {
        for (String auto : new String[] {"two-tip", "two-tip-blue"}) {
            Json steps = read(new File(root(), "autos/" + auto + ".auto.json")).get("steps");
            int down = -1;
            for (int i = 0; i < steps.size(); i++) {
                if ("flowerBarDownFlower".equals(steps.at(i).str("id"))) down = i;
            }
            assertEquals("waitBarDown", steps.at(down + 1).str("id"), auto + ": the wait follows the bar drop");
            assertEquals(MechanismConstants.FLOWER_BAR_DROP_S, steps.at(down + 1).num("seconds"), 1e-9,
                    auto + ": the wait is FLOWER_BAR_DROP_S");
            assertEquals("collectFlower", steps.at(down + 2).str("id"), auto + ": then the drive in");
        }
    }

    private static Json read(File f) throws IOException {
        try (InputStream in = new FileInputStream(f)) {
            return Json.parse(in, f.getName());
        }
    }

    private static File root() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) dir = dir.getParentFile();
        if (dir == null) throw new IllegalStateException("no zenith.json above " + System.getProperty("user.dir"));
        return dir;
    }
}
