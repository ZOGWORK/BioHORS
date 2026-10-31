package org.firstinspires.ftc.teamcode.aim;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.SimElapsedTime;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/** The auto-to-teleop pose expires after MAX_POSE_AGE_S and is taken only once. */
class PoseHandoffTest {

    @AfterEach
    void clear() {
        OpMode.blackboard.clear();
    }

    /** As PoseHandoff.save, with a timer on a clock the test controls. */
    private static FakeTimer savedOn() {
        FakeTimer clock = new FakeTimer();
        OpMode.blackboard.put("biohors.alliance", "RED");
        OpMode.blackboard.put("biohors.pose", new double[] {-61.5, 49.0, 0.0});
        OpMode.blackboard.put("biohors.poseTimer", new SimElapsedTime(clock));
        return clock;
    }

    @Test
    void aFreshPoseIsHandedOver() {
        FakeTimer clock = savedOn();
        clock.advance(8.0); // the AUTO-to-teleop transition
        assertArrayEquals(new double[] {-61.5, 49.0, 0.0}, PoseHandoff.savedPose(), 1e-12);
        assertEquals(8.0, PoseHandoff.savedPoseAgeS(), 1e-9);
    }

    @Test
    void aPoseOlderThanTheLimitIsNot() {
        FakeTimer clock = savedOn();
        clock.advance(PoseHandoff.MAX_POSE_AGE_S + 0.1);
        assertNull(PoseHandoff.savedPose(), "a practice auto from long ago must not place the robot");
        assertEquals(Alliance.RED, PoseHandoff.savedAlliance(), "the alliance is still remembered");
    }

    @Test
    void aPoseWithNoTimerIsNotTrusted() {
        OpMode.blackboard.put("biohors.pose", new double[] {1.0, 2.0, 3.0});
        assertNull(PoseHandoff.savedPose());
        assertTrue(Double.isNaN(PoseHandoff.savedPoseAgeS()));
    }

    @Test
    void clearingForgetsThePoseButKeepsTheAlliance() {
        savedOn();
        PoseHandoff.clearPose();
        assertNull(PoseHandoff.savedPose());
        assertTrue(Double.isNaN(PoseHandoff.savedPoseAgeS()));
        assertEquals(Alliance.RED, PoseHandoff.savedAlliance());
    }

    @Test
    void saveStampsThePoseNow() {
        PoseHandoff.save(Alliance.BLUE, 61.5, -49.0, Math.PI);
        assertArrayEquals(new double[] {61.5, -49.0, Math.PI}, PoseHandoff.savedPose(), 1e-12);
        assertTrue(PoseHandoff.savedPoseAgeS() < 1.0, "just saved");
    }
}
