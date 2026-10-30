package org.firstinspires.ftc.teamcode.auto;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.junit.jupiter.api.Test;

/** The gate's interlock: only at the alliance's own scoring spots, within the tolerances. */
class ShootingPoseTest {

    @Test
    void atBothSpotsOnEachAlliance() {
        for (Alliance a : Alliance.values()) {
            assertTrue(ShootingPose.at(a, AutoPoses.of(a, AutoPoses.SCORE_SOUTH)), a + " scoreSouth");
            assertTrue(ShootingPose.at(a, AutoPoses.of(a, AutoPoses.SCORE_NORTH)), a + " scoreNorth");
        }
    }

    @Test
    void notOffTheSpotOrTurnedAway() {
        Pose s = AutoPoses.SCORE_SOUTH;
        assertFalse(ShootingPose.at(Alliance.RED, new Pose(s.x(), s.y() - 3.0, s.heading())), "3 in short");
        assertFalse(ShootingPose.at(Alliance.RED, new Pose(s.x(), s.y(), s.heading() + Math.toRadians(5))), "turned 5 deg");
        assertFalse(ShootingPose.at(Alliance.RED, AutoPoses.START), "the start pose");
        assertFalse(ShootingPose.at(Alliance.BLUE, s), "RED's spot as BLUE");
    }

    @Test
    void withinTheTolerance() {
        Pose s = AutoPoses.SCORE_NORTH;
        assertTrue(ShootingPose.at(Alliance.RED, new Pose(s.x() + 1.0, s.y() - 1.0, s.heading() + Math.toRadians(2))));
    }
}
