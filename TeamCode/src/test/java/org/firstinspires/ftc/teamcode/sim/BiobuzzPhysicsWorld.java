package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.field.ConvexShape;
import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiecePhysicsConfig;
import org.horizon36596.simloop.field.KinematicBody;
import org.horizon36596.simloop.field.ObstacleBox;
import org.horizon36596.simloop.field.OrientedBox;
import org.horizon36596.simloop.field.RobotShape;
import org.horizon36596.zenith.Json;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * The BIOBUZZ field as SimLoop's game-piece physics sees it, for BiobuzzMatch's physics mode and the
 * tests that check it. Everything here is SimLoop's own model (field package); nothing is a physics
 * engine of ours:
 *  - both hives as hinged bodies (BiobuzzHive), each on its kickoff stop;
 *  - each hive's frame (its floor bars and four legs, from the field file's obstacles) as a static
 *    body, so a shot that hits it bounces off;
 *  - the flowers' feet as obstacles;
 *  - the walls at the field file's size.
 * Pieces collide with each other (on the floor and in the air) and with all of these using SimLoop's
 * contact numbers (GamePiecePhysicsConfig defaults: CARRIED OVER for a foam ball, not measured on
 * pollen). The robot is solid to them (robotShape): it pushes loose pollen it drives into.
 */
public final class BiobuzzPhysicsWorld {

    /** PLACEHOLDER: 1 ms physics sub-steps (SimLoop's default is 5 ms), for contact that looks
     *  continuous in a replay. Changes no measured number; NEEDS MEASUREMENT only in the sense that
     *  any sub-step this small gives the same motion. */
    static final double SUBSTEP_S = 0.001;
    /** APPROX: a frame leg (a 1 in square tube, field file strut) as this many boxes along its length. */
    static final int LEG_SEGMENTS = 16;
    /** APPROX (CAD footprint numbers, robot.json): the flower mech's side plates beside the mouth stand
     *  this tall: what a rolling pollen meets beside the mouth. */
    static final double PLATE_HEIGHT_IN = 6.0;
    /** MEASURED FROM CAD (AdvantageScope's field model, am-5852 Nectar: 3.6 in across every way). */
    static final double NECTAR_RADIUS_IN = 1.8;
    /** CALIBRATED in StagedNectarTest (pollen = 1): the middle of 1.50-1.70, where the 3 nectar staged
     *  in an up cell make 3 shot pollen (75 deg, MEASURED FROM CAD) tip it, as the season's tip table
     *  says. Not a weighed nectar. */
    static double NECTAR_MASS = 1.60;

    private BiobuzzPhysicsWorld() {
    }

    public static GamePiecePhysicsConfig physics() {
        return GamePiecePhysicsConfig.builder().pieceMass("pollen", 1.0).pieceMass("nectar", NECTAR_MASS)
                .maxSubstepSeconds(SUBSTEP_S).build();
    }

    /** The field: walls, both hives, both hive frames, the flower feet. */
    public static FieldWorld world(Json field) {
        double half = field.get("sizeIn").num("xIn") / 2.0;
        FieldWorld.Builder b = FieldWorld.builder().perimeter(half, half).physics(physics())
                .kinematicBody(BiobuzzHive.body(true)).kinematicBody(BiobuzzHive.body(false));
        List<ConvexShape> frameRed = new ArrayList<>(), frameBlue = new ArrayList<>();
        for (Json o : field.get("obstacles").items()) {
            String id = o.str("id");
            String kind = o.str("kind");
            if (id.startsWith("frame")) {
                // The frame bars and legs: the west ones hold RED's hive (x < 0), the east ones BLUE's.
                List<ConvexShape> into = id.contains("West") ? frameRed : frameBlue;
                if ("box".equals(kind)) {
                    into.add(ConvexShape.box(id, o.num("minXIn"), o.num("maxXIn"), o.num("minYIn"), o.num("maxYIn"),
                            o.num("minZIn"), o.num("maxZIn")));
                } else if ("strut".equals(kind)) {
                    into.addAll(strut(id, o.get("fromIn"), o.get("toIn"), o.num("radiusIn")));
                }
            } else if (id.startsWith("flower") && id.endsWith("Foot") && "box".equals(kind)) {
                b.obstacle(new ObstacleBox(id, o.num("minXIn"), o.num("maxXIn"), o.num("minYIn"), o.num("maxYIn"),
                        o.num("minZIn"), o.num("maxZIn")));
            }
        }
        b.kinematicBody(frame("hiveFrameRed", frameRed));
        b.kinematicBody(frame("hiveFrameBlue", frameBlue));
        return b.build();
    }

    /**
     * The 3 nectar staged in a hive's up cell at kickoff (field file: each hive holds 3), set down
     * spread across the cell's width at its open end, to settle at the back as the field is set.
     */
    /** Returns the 3 nectar's piece ids. */
    public static int[] stageNectar(org.horizon36596.simloop.field.GamePieceTracker t, boolean red, boolean north) {
        String hive = BiobuzzHive.bodyName(red);
        double y = (north ? 1 : -1) * (BiobuzzHive.OUTER_FACE_Y_IN - NECTAR_RADIUS_IN - 0.5);
        double[] f = new double[3];
        int[] ids = new int[3];
        for (int i = 0; i < 3; i++) {
            double x = -6.0 + 6.0 * i;
            t.getWorld().getKinematicBody(hive).toField(t.getBodyAngleRad(hive), x, y, NECTAR_RADIUS_IN + 0.3, f);
            ids[i] = t.addPieceAt("nectar", f[0], f[1], f[2], NECTAR_RADIUS_IN);
        }
        return ids;
    }

    /** A static body (no hinge, never turned): pieces bounce off its shapes, nothing moves it. */
    private static KinematicBody frame(String name, List<ConvexShape> shapes) {
        if (shapes.isEmpty()) throw new IllegalStateException(name + ": no frame obstacles in the field file");
        return new KinematicBody(name, new double[] {0, 0, 0}, new double[] {1, 0, 0}, shapes,
                Collections.emptyList(), BiobuzzHive.RESTITUTION, BiobuzzHive.FRICTION);
    }

    /** A round strut as LEG_SEGMENTS axis-aligned boxes along it, each its radius around its axis. */
    private static List<ConvexShape> strut(String id, Json from, Json to, double r) {
        double x0 = from.num("xIn"), y0 = from.num("yIn"), z0 = from.num("zIn");
        double x1 = to.num("xIn"), y1 = to.num("yIn"), z1 = to.num("zIn");
        List<ConvexShape> out = new ArrayList<>();
        for (int k = 0; k < LEG_SEGMENTS; k++) {
            double a = (double) k / LEG_SEGMENTS, c = (double) (k + 1) / LEG_SEGMENTS;
            double xa = x0 + a * (x1 - x0), xc = x0 + c * (x1 - x0);
            double ya = y0 + a * (y1 - y0), yc = y0 + c * (y1 - y0);
            double za = z0 + a * (z1 - z0), zc = z0 + c * (z1 - z0);
            out.add(ConvexShape.box(id + k, Math.min(xa, xc) - r, Math.max(xa, xc) + r, Math.min(ya, yc) - r,
                    Math.max(ya, yc) + r, Math.max(0, Math.min(za, zc) - r), Math.max(za, zc) + r));
        }
        return out;
    }

    /**
     * The robot as solid boxes, robot frame (x forward, y left, z up from the floor; robot.json):
     * the chassis from its back to the frame front, and the flower mech's two side plates beside the
     * intake mouth. The mouth itself is open, so a pollen reaches the intake trigger there; anywhere
     * else the robot pushes it.
     */
    public static RobotShape robotShape(Json robot) {
        return robotShape(robot, false);
    }

    /**
     * As robotShape(robot), with the flower bar stowed: the bar and its side plates fold back inside
     * the frame (MechanismConstants.FLOWER_BAR_STOWED), so only the chassis is left to meet a pollen.
     */
    public static RobotShape robotShape(Json robot, boolean barStowed) {
        Json fp = robot.get("footprint").get("expandedIn");
        double length = fp.num("lengthIn"), width = fp.num("widthIn"), height = robot.get("heightIn").num("value");
        double cor = robot.get("footprint").get("centreOfRotationIn").num("xIn");
        double back = -cor - length / 2.0;                      // robot.json: the box with the bar stowed
        double frameFront = BiobuzzMatch.FRAME_FRONT_IN, tip = BiobuzzMatch.FLOWER_BAR_TIP_IN;
        double mouthWidth = robot.get("mouths").at(0).num("widthIn");
        OrientedBox chassis = OrientedBox.axisAligned("chassis", (back + frameFront) / 2.0, 0, height / 2.0,
                (frameFront - back) / 2.0, width / 2.0, height / 2.0);
        if (barStowed) return new RobotShape(Collections.singletonList(chassis));
        // The bar out: its two side plates beside the opening, from the frame front to the bar tip.
        double plateWidth = (width - mouthWidth) / 2.0;
        double plateY = mouthWidth / 2.0 + plateWidth / 2.0;
        double plateX = (frameFront + tip) / 2.0, plateHalfX = (tip - frameFront) / 2.0;
        return new RobotShape(Arrays.asList(chassis,
                OrientedBox.axisAligned("plateLeft", plateX, plateY, PLATE_HEIGHT_IN / 2.0,
                        plateHalfX, plateWidth / 2.0, PLATE_HEIGHT_IN / 2.0),
                OrientedBox.axisAligned("plateRight", plateX, -plateY, PLATE_HEIGHT_IN / 2.0,
                        plateHalfX, plateWidth / 2.0, PLATE_HEIGHT_IN / 2.0)));
    }
}
