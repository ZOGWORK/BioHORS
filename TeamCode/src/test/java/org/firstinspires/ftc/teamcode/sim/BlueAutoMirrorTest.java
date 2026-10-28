package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * The BLUE auto file is Zenith's point mirror of the RED one (tools/mirror_auto.mjs writes it), in
 * the source folder and the deployed copy alike. Fails when someone edits the RED auto or a waypoint
 * and forgets to re-run the script, or edits the BLUE file by hand.
 *
 * Rule (biobuzz.field.json: pointSymmetry): x and y negate, every absolute heading (headingRad,
 * fromRad, toRad) turns by pi, everything else (turns, path fractions, speeds, waits, counts) is
 * equal, and a field id swaps Red for Blue. A RED pose that names a waypoint is that waypoint's pose.
 */
class BlueAutoMirrorTest {

    private static final String AUTO = "two-tip";
    private static final Set<String> TEXT = Set.of("notes", "description", "name", "title", "provenance", "id",
            "$schema", "authors", "created");
    private static final double EPS = 1e-6;

    @Test
    void theBlueAutoIsTheMirrorOfTheRedOne() throws IOException {
        File root = projectRoot();
        for (String dir : new String[] {"autos", "TeamCode/src/main/assets/autos"}) {
            Json red = read(new File(root, dir + "/" + AUTO + ".auto.json"));
            Json blue = read(new File(root, dir + "/" + AUTO + "-blue.auto.json"));
            Json waypoints = read(new File(root, "autos/waypoints.json")).get("waypoints");
            assertEquals("RED", red.str("alliance"));
            assertEquals("BLUE", blue.str("alliance"));
            List<String> bad = new ArrayList<>();
            compare(red, blue, waypoints, "", "", bad);
            assertTrue(bad.isEmpty(), dir + ": the BLUE auto is not the mirror of the RED one; re-run "
                    + "`node tools/mirror_auto.mjs` and `npx zenith deploy`:\n  " + String.join("\n  ", bad));
        }
    }

    private static void compare(Json r, Json b, Json waypoints, String key, String at, List<String> bad) {
        if (r.isObject() && r.has("ref") && r.get("ref").isString()) {
            Json w = waypoints.get(r.str("ref"));
            for (String k : new String[] {"xIn", "yIn", "headingRad"}) {
                compare(w.get(k), b.get(k), waypoints, k, at + "." + k, bad);
            }
            return;
        }
        if (r.isObject()) {
            for (String k : r.keys()) {
                if (TEXT.contains(k)) continue;
                if (!b.has(k)) {
                    bad.add(at + "." + k + " missing");
                    continue;
                }
                compare(r.get(k), b.get(k), waypoints, k, at + "." + k, bad);
            }
        } else if (r.isArray()) {
            if (r.size() != b.size()) {
                bad.add(at + ": " + r.size() + " vs " + b.size() + " items");
                return;
            }
            for (int i = 0; i < r.size(); i++) compare(r.at(i), b.at(i), waypoints, key, at + "[" + i + "]", bad);
        } else if (r.isNumber()) {
            double x = r.num(), y = b.num();
            boolean ok;
            if (key.equals("xIn") || key.equals("yIn")) {
                ok = Math.abs(x + y) < EPS;
            } else if (key.equals("headingRad") || key.equals("fromRad") || key.equals("toRad")) {
                double d = Math.IEEEremainder(y - x - Math.PI, 2 * Math.PI);
                ok = Math.abs(d) < EPS;
            } else {
                ok = Math.abs(x - y) < EPS;
            }
            if (!ok) bad.add(at + ": RED " + x + ", BLUE " + y);
        } else if (r.isString()) {
            String want = r.text().replace("RED", "\u0000").replace("BLUE", "RED").replace("\u0000", "BLUE")
                    .replace("Red", "\u0000").replace("Blue", "Red").replace("\u0000", "Blue");
            if (!want.equals(b.text())) bad.add(at + ": RED \"" + r.text() + "\", BLUE \"" + b.text() + "\"");
        }
    }

    private static Json read(File f) throws IOException {
        try (InputStream in = new FileInputStream(f)) {
            return Json.parse(in, f.getName());
        }
    }

    private static File projectRoot() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) dir = dir.getParentFile();
        if (dir == null) throw new IllegalStateException("no zenith.json above " + System.getProperty("user.dir"));
        return dir;
    }
}
