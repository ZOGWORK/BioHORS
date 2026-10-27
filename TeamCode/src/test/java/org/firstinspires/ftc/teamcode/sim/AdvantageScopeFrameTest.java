package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.wpi.math.Pose2d;

/**
 * Positions and headings must turn by the same rotation, or the robot is drawn facing backward.
 * A robot that drives "forward" for 1 in must move along the direction its drawn heading points.
 */
class AdvantageScopeFrameTest {

    @Test
    void aRobotMovesTheWayItIsDrawnFacing() {
        double[][] cases = {{0, 0, 0}, {-13, -63, Math.PI / 2}, {20, 5, -2.0}, {-40, 30, 3.0}};
        for (double[] c : cases) {
            Pose2d a = AdvantageScopeFrame.pose2d(c[0], c[1], c[2]);
            Pose2d b = AdvantageScopeFrame.pose2d(c[0] + Math.cos(c[2]), c[1] + Math.sin(c[2]), c[2]);
            double moved = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
            double drawn = a.getRotation().getRadians();
            assertEquals(0.0, Math.atan2(Math.sin(moved - drawn), Math.cos(moved - drawn)), 1e-9,
                    "pose " + java.util.Arrays.toString(c));
        }
    }

    @Test
    void theRedStartFacingTheHiveIsDrawnFacingAwayFromTheRedWall() {
        // RED start (-13, -63) facing +Y (north, toward the hive). AdvantageScope Center/Rotated:
        // their +X is our -Y, so our +Y (north) is their -X: heading 180 deg.
        Pose2d p = AdvantageScopeFrame.pose2d(-13, -63, Math.PI / 2);
        assertEquals(63 * 0.0254, p.getX(), 1e-9);
        assertEquals(-13 * 0.0254, p.getY(), 1e-9);
        assertEquals(Math.PI, Math.abs(p.getRotation().getRadians()), 1e-9);
    }
}
