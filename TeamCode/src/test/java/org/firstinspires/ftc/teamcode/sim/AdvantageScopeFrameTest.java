package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Pose3d;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

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
        // RED start (-15, -64.2) facing +Y (north, toward the hive). AdvantageScope Center/Rotated:
        // their +X is our -Y, so our +Y (north) is their -X: heading 180 deg.
        Pose2d p = AdvantageScopeFrame.pose2d(-15, -64.2, Math.PI / 2);
        assertEquals(64.2 * 0.0254, p.getX(), 1e-9);
        assertEquals(-15 * 0.0254, p.getY(), 1e-9);
        assertEquals(Math.PI, Math.abs(p.getRotation().getRadians()), 1e-9);
    }

    /**
     * Our field file and AdvantageScope's field model agree where things are. AdvantageScope reads a
     * logged Center/Rotated pose (X, Y) and draws it in its model at (Y, -X), the model's own axes
     * (checked by eye in its 2D Field tab: the red garden is drawn on the model's red garden). So
     * each garden pollen and flower of the field file, logged and turned that way, must land on one
     * of the model's staged pollen (within 2 in). Read from the model's own GLB when AdvantageScope is
     * installed here; skipped otherwise.
     */
    @Test
    void ourFieldLinesUpWithAdvantageScopesFieldModel() throws IOException {
        File glb = new File(System.getenv().getOrDefault("APPDATA", ""),
                "AdvantageScope/autoAssets/Field3d_20262027FTCFieldV1/model.glb");
        org.junit.jupiter.api.Assumptions.assumeTrue(glb.isFile(), "AdvantageScope's 2026-2027 field is not installed");
        java.util.List<double[]> staged = StagedPollen.read(glb);
        Json field;
        try (InputStream in = new FileInputStream(new File(projectRoot(), "autos/field/biobuzz.field.json"))) {
            field = Json.parse(in, "biobuzz.field.json");
        }
        int checked = 0;
        for (Json e : field.get("elements").items()) {
            String kind = e.optText("kind", "");
            boolean flower = kind.equals("container") && "flower".equals(e.optText("container", ""));
            boolean garden = kind.equals("pollen") && e.str("id").startsWith("garden");
            if (!flower && !garden) continue;
            Pose2d p = AdvantageScopeFrame.pose2d(e.num("xIn"), e.num("yIn"), 0);
            double modelX = p.getY(), modelY = -p.getX();   // Center/Rotated -> the model's axes
            double best = Double.MAX_VALUE;
            for (double[] s : staged) best = Math.min(best, Math.hypot(s[0] - modelX, s[1] - modelY));
            assertTrue(best < 2 * 0.0254, e.str("id") + " is " + best / 0.0254 + " in from the model's nearest pollen");
            checked++;
        }
        assertTrue(checked >= 12, "checked " + checked);
    }

    /**
     * AdvantageScope moves each flywheel model by its config.json zeroedPosition, then by the logged
     * Field/RobotComponents pose. With no spin the two must cancel, or the flywheel is drawn away
     * from its shooter (and orbits instead of spinning in place).
     */
    @Test
    void theFlywheelComponentsSpinWhereTheModelPutsThem() throws IOException {
        File dir = new File(projectRoot(), "advantagescope/Robot_BioHORS");
        for (String f : new String[] {"model.glb", "model_0.glb", "model_1.glb"}) {
            assertTrue(new File(dir, f).isFile(), "missing " + f);
        }
        Json components;
        try (InputStream in = new FileInputStream(new File(dir, "config.json"))) {
            components = Json.parse(in, "config.json").get("components");
        }
        assertEquals(2, components.size(), "model_0 pollen and model_1 nectar");
        Pose3d[] logged = {
            AdvantageScopeFrame.robotFlywheel(BiobuzzMatch.POLLEN_AXLE_FORWARD_IN, BiobuzzMatch.POLLEN_AXLE_UP_IN, 0),
            AdvantageScopeFrame.robotFlywheel(BiobuzzMatch.NECTAR_AXLE_FORWARD_IN, BiobuzzMatch.NECTAR_AXLE_UP_IN, 0)};
        for (int i = 0; i < 2; i++) {
            Json zero = components.at(i).get("zeroedPosition");
            assertEquals(0.0, logged[i].getX() + zero.at(0).num(), 1e-5, "component " + i + " x");
            assertEquals(0.0, logged[i].getY() + zero.at(1).num(), 1e-5, "component " + i + " y");
            assertEquals(0.0, logged[i].getZ() + zero.at(2).num(), 1e-5, "component " + i + " z");
        }
    }

    private static File projectRoot() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) dir = dir.getParentFile();
        if (dir == null) throw new IllegalStateException("no zenith.json above " + System.getProperty("user.dir"));
        return dir;
    }
}
