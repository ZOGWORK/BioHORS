package org.firstinspires.ftc.teamcode.aim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.junit.jupiter.api.Test;

class DistanceSpeedTableTest {

    private static final double FALLBACK = 1234.0;
    private final DistanceSpeedTable table = new DistanceSpeedTable(
            new double[] {30.0, 50.0, 70.0}, new double[] {1500.0, 1700.0, 2100.0});

    @Test
    void interpolatesInAStraightLineBetweenPoints() {
        assertEquals(1600.0, table.rpmAt(40.0, FALLBACK), 1e-9, "halfway between 30 and 50 in");
        assertEquals(1800.0, table.rpmAt(55.0, FALLBACK), 1e-9, "a quarter of the way from 50 to 70 in");
        assertEquals(1700.0, table.rpmAt(50.0, FALLBACK), 1e-9, "on a point, that point's rpm");
    }

    @Test
    void holdsTheEndPointsOutsideTheTable() {
        assertEquals(1500.0, table.rpmAt(5.0, FALLBACK), 1e-9, "closer than the first point");
        assertEquals(2100.0, table.rpmAt(140.0, FALLBACK), 1e-9, "farther than the last point");
    }

    @Test
    void aOnePointTableIsAConstant() {
        DistanceSpeedTable one = new DistanceSpeedTable(new double[] {48.0}, new double[] {1600.0});
        for (double d : new double[] {0.0, 20.0, 48.0, 100.0}) {
            assertEquals(1600.0, one.rpmAt(d, FALLBACK), 1e-9, "at " + d + " in");
        }
    }

    @Test
    void theSeededTablesEqualEachShootersFixedTarget() {
        DistanceSpeedTable pollen = new DistanceSpeedTable(
                MechanismConstants.POLLEN_SPEED_TABLE_DISTANCE_IN, MechanismConstants.POLLEN_SPEED_TABLE_RPM);
        DistanceSpeedTable nectar = new DistanceSpeedTable(
                MechanismConstants.NECTAR_SPEED_TABLE_DISTANCE_IN, MechanismConstants.NECTAR_SPEED_TABLE_RPM);
        for (double d : new double[] {10.0, 48.0, 90.0}) {
            assertEquals(MechanismConstants.POLLEN_SHOOTER.targetRpm, pollen.rpmAt(d, -1.0), 1e-9);
            assertEquals(MechanismConstants.NECTAR_SHOOTER.targetRpm, nectar.rpmAt(d, -1.0), 1e-9);
        }
    }

    @Test
    void aNonFiniteDistanceGivesTheFallback() {
        assertEquals(FALLBACK, table.rpmAt(Double.NaN, FALLBACK), 1e-9);
        assertEquals(FALLBACK, table.rpmAt(Double.POSITIVE_INFINITY, FALLBACK), 1e-9);
        assertEquals(FALLBACK, table.rpmAt(Double.NEGATIVE_INFINITY, FALLBACK), 1e-9);
    }

    @Test
    void aMalformedTableIsUnusableAndGivesTheFallback() {
        DistanceSpeedTable[] bad = {
                new DistanceSpeedTable(new double[0], new double[0]),
                new DistanceSpeedTable(null, new double[] {1600.0}),
                new DistanceSpeedTable(new double[] {30.0, 50.0}, new double[] {1500.0}),
                new DistanceSpeedTable(new double[] {50.0, 30.0}, new double[] {1500.0, 1700.0}),
                new DistanceSpeedTable(new double[] {30.0, 30.0}, new double[] {1500.0, 1700.0}),
                new DistanceSpeedTable(new double[] {30.0, Double.NaN}, new double[] {1500.0, 1700.0}),
                new DistanceSpeedTable(new double[] {30.0, 50.0}, new double[] {1500.0, Double.POSITIVE_INFINITY}),
                new DistanceSpeedTable(new double[] {30.0, 50.0}, new double[] {-10.0, 1700.0}),
        };
        for (DistanceSpeedTable t : bad) {
            assertFalse(t.isUsable());
            assertEquals(FALLBACK, t.rpmAt(40.0, FALLBACK), 1e-9);
        }
        assertTrue(table.isUsable());
    }

    @Test
    void laterEditsToTheSourceArraysDoNotChangeATable() {
        double[] d = {30.0, 50.0};
        double[] r = {1500.0, 1700.0};
        DistanceSpeedTable t = new DistanceSpeedTable(d, r);
        r[0] = 9999.0;
        assertEquals(1500.0, t.rpmAt(30.0, FALLBACK), 1e-9);
    }
}
