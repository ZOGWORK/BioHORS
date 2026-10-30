package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiecePhysicsConfig;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.junit.jupiter.api.Test;

import java.util.Locale;

/**
 * The 3 nectar staged in each hive's up cell at kickoff, as real pieces in the physics hive: with
 * them in, the season's tip table says 3 pollen tip the cell (rules.tipTable, [8, 7, 6, 3, 1, 0]).
 * The hive's hold is calibrated to 8 pollen in an empty cell (BiobuzzHiveTest), so the nectar's
 * mass is CALIBRATED here to that row: 3 nectar + 3 pollen tip it, 3 nectar + 2 do not.
 */
class StagedNectarTest {

    /** How many pollen, on top of 3 staged nectar, tipped RED's hive from south-up; -1 if 12 did not. */
    static int pollenToTipWithNectar(double nectarMass) {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder().pieceMass("pollen", 1.0)
                .pieceMass("nectar", nectarMass).maxSubstepSeconds(BiobuzzPhysicsWorld.SUBSTEP_S).build();
        GamePieceTracker t = new GamePieceTracker(1, FieldWorld.builder().perimeter(72, 72).physics(physics)
                .kinematicBody(BiobuzzHive.body(true)).build());
        String hive = BiobuzzHive.bodyName(true);
        t.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
        BiobuzzPhysicsWorld.stageNectar(t, true, false);
        BiobuzzHiveTest.run(t, 3.0);
        if (t.getBodyAngleRad(hive) > 0.0) return 0;
        for (int n = 1; n <= 12; n++) {
            BiobuzzHiveTest.dropIntoUpCell(t, true, false, n);
            BiobuzzHiveTest.run(t, 6.0);
            if (t.getBodyAngleRad(hive) > 0.0) return n;
        }
        return -1;
    }

    @Test
    void calibrationSweepIsPrinted() {
        for (double m = 1.3; m <= 1.91; m += 0.05) {
            double saved = BiobuzzPhysicsWorld.NECTAR_MASS;
            BiobuzzPhysicsWorld.NECTAR_MASS = m;
            try {
                System.out.printf(Locale.ROOT, "NECTAR mass %.2f: placed %d, shot %d%n", m, pollenToTipWithNectar(m),
                        BiobuzzHiveTest.shotsToTip(BiobuzzHive.COM_HEIGHT_IN, true));
            } finally {
                BiobuzzPhysicsWorld.NECTAR_MASS = saved;
            }
        }
    }

    @Test
    void withTheThreeStagedNectarThreePollenTipIt() {
        assertEquals(3, BiobuzzHiveTest.shotsToTip(BiobuzzHive.COM_HEIGHT_IN, true), "the season table's 3-nectar row");
    }
}
