package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.zenith.Json;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

/**
 * Reads where AdvantageScope's field model stages its pollen: the world position of every node
 * named "...Pollen" in the model's GLB, turned by the field config's +90 deg about x (GLB y-up to
 * field z-up), in metres {x, y, z}. For AdvantageScopeFrameTest only.
 */
final class StagedPollen {

    private StagedPollen() {
    }

    static List<double[]> read(File glb) throws IOException {
        byte[] b = Files.readAllBytes(glb.toPath());
        int jsonLen = ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN).getInt(12);
        Json gltf = Json.parse(new ByteArrayInputStream(b, 20, jsonLen), "model.glb");
        Json nodes = gltf.get("nodes");
        List<double[]> out = new ArrayList<>();
        Json scene = gltf.get("scenes").at(0).get("nodes");
        for (int i = 0; i < scene.size(); i++) walk(nodes, scene.at(i).integer(), identity(), out);
        return out;
    }

    private static void walk(Json nodes, int i, double[] parent, List<double[]> out) {
        Json n = nodes.at(i);
        double[] m = mul(parent, local(n));
        if (n.optText("name", "").endsWith("Pollen")) out.add(new double[] {m[3], -m[11], m[7]});
        Json kids = n.opt("children");
        if (kids != null) for (int k = 0; k < kids.size(); k++) walk(nodes, kids.at(k).integer(), m, out);
    }

    /** The node's local transform, row-major 4x4. */
    private static double[] local(Json n) {
        Json mat = n.opt("matrix");
        if (mat != null) {
            double[] r = new double[16];
            for (int row = 0; row < 4; row++) {
                for (int col = 0; col < 4; col++) r[row * 4 + col] = mat.at(col * 4 + row).num();
            }
            return r;
        }
        double[] t = vec(n.opt("translation"), new double[] {0, 0, 0});
        double[] q = vec(n.opt("rotation"), new double[] {0, 0, 0, 1});
        double[] s = vec(n.opt("scale"), new double[] {1, 1, 1});
        double x = q[0], y = q[1], z = q[2], w = q[3];
        double[][] r = {
                {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
        double[] m = identity();
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) m[row * 4 + col] = r[row][col] * s[col];
            m[row * 4 + 3] = t[row];
        }
        return m;
    }

    private static double[] vec(Json j, double[] dflt) {
        if (j == null) return dflt;
        double[] v = new double[j.size()];
        for (int k = 0; k < v.length; k++) v[k] = j.at(k).num();
        return v;
    }

    private static double[] identity() {
        return new double[] {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    }

    private static double[] mul(double[] a, double[] b) {
        double[] r = new double[16];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                double s = 0;
                for (int k = 0; k < 4; k++) s += a[i * 4 + k] * b[k * 4 + j];
                r[i * 4 + j] = s;
            }
        }
        return r;
    }
}
