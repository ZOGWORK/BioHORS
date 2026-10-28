package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.field.BodyCell;
import org.horizon36596.simloop.field.ConvexShape;
import org.horizon36596.simloop.field.HingeDynamics;
import org.horizon36596.simloop.field.KinematicBody;

import java.util.Arrays;
import java.util.List;

/**
 * A BIOBUZZ hive as a SimLoop physics body: a bar on an axle along field X, a closed cell at each
 * end (open only at its OUTER face, where a launch enters), resting on a 30 deg stop with one
 * cell up. Its own weight holds it on the stop (centre of mass above the axle); pollen sitting in
 * the up cell's far end push it over. When it tips, the old up cell swings down, slopes outward,
 * and its pollen rolls out of the open end onto the floor: the spill.
 *
 * Local frame (before the tilt): x along the axle, y along the bar (cells at -y and +y), z up.
 * The body angle turns about +X; at +30 deg the +y (north) cell is up, at -30 deg the south one.
 *
 * Geometry, in the bar's own (untilted) frame: SPEC where the field file / manual gives it, else
 * APPROX fitted to it:
 *   - pivot x = +/-12.75 (SPEC, field file); the bar is 20 in wide in x (SPEC, the pivot-bar box).
 *   - along the bar: the cells' inner faces 18.84 in apart and each cell 12.04 in deep (SPEC, manual
 *     Fig 9-9, as the field file quotes it), so a cell runs from 9.42 to 21.46 in from the axle. At the
 *     30 deg stop the outer face is 21.46 cos 30 = 18.58 in from the pivot in plan, as the field file says.
 *   - axle height and cell height APPROX: chosen so the up cell's opening spans about 53.5 to 65.6 in
 *     (manual Fig 9-10) at the 30 deg stop: the floor lip is 21.46 sin 30 = 10.73 in above the axle.
 *   - hold (mass, centre of mass, inertia, damping) CALIBRATED in BiobuzzHiveTest so that 8 pollen
 *     shot into the up cell (ShotModel, as the auto fires them) tip it and 7 do not (the rule for a cell
 *     with no nectar), and the swing takes a few seconds. Shot pollen roll back toward the axle, so they
 *     tip it later than pollen set down at the cell's mouth would.
 */
public final class BiobuzzHive {

    public static final double TILT_RAD = Math.toRadians(30.0);   // SPEC (manual: the bar rests at 30 deg)
    public static final double AXLE_Z_IN = 42.77;                  // APPROX: 53.5 - 10.73 (the opening's bottom)
    public static final double HALF_WIDTH_X_IN = 10.0;             // SPEC (20 in pivot bar)
    public static final double OUTER_FACE_Y_IN = 21.46;            // SPEC: 18.84 / 2 + 12.04, along the bar
    public static final double INNER_WALL_Y_IN = 9.42;             // SPEC: the inner faces are 18.84 in apart
    public static final double CELL_HEIGHT_IN = 14.0;              // APPROX: (65.6 - 53.5) / cos 30
    public static final double WALL_IN = 0.75;                     // APPROX: wall thickness

    // Ball-on-hive contact: APPROX (SimLoop's example values), not measured for pollen on a real hive.
    public static double RESTITUTION = 0.15;
    public static double FRICTION = 0.6;

    // Hold: CALIBRATED in BiobuzzHiveTest (8 pollen tip, 7 do not). Mass in pollen units.
    public static double HIVE_MASS = 20.0;          // CALIBRATED
    public static double COM_HEIGHT_IN = 5.6;       // CALIBRATED: mid-band of 5.2..6.0 in, where the 8th real shot tips it (BiobuzzHiveTest, at this inertia)
    public static double INERTIA = 22000.0;         // CALIBRATED: with the hold above the swing at the tip load takes about 2.7 s (DSim: about 4 s, APPROX)
    public static double DAMPING_PER_S = 0.6;       // CALIBRATED

    private BiobuzzHive() {
    }

    public static String bodyName(boolean red) {
        return red ? "hiveRed" : "hiveBlue";
    }

    public static String cellName(boolean red, boolean north) {
        return bodyName(red) + (north ? "North" : "South");
    }

    /** The hive body, its up cell set by the angle the tracker is given (setBodyMotion). */
    public static KinematicBody body(boolean red) {
        double px = red ? -12.75 : 12.75;
        String n = bodyName(red);
        double w = HALF_WIDTH_X_IN, o = OUTER_FACE_Y_IN, i = INNER_WALL_Y_IN, h = CELL_HEIGHT_IN, t = WALL_IN;
        List<ConvexShape> shapes = Arrays.asList(
                ConvexShape.box(n + "Floor", -w, w, -o, o, -t, 0),
                ConvexShape.box(n + "Roof", -w, w, -o, o, h, h + t),
                ConvexShape.box(n + "Hub", -w, w, -i, i, 0, h),
                ConvexShape.box(n + "SideWest", -w - t, -w, -o, o, -t, h + t),
                ConvexShape.box(n + "SideEast", w, w + t, -o, o, -t, h + t));
        List<BodyCell> cells = Arrays.asList(
                new BodyCell(cellName(red, true), -w, w, i, o, 0, h, 1.0),
                new BodyCell(cellName(red, false), -w, w, -o, -i, 0, h, 1.0));
        KinematicBody body = new KinematicBody(n, new double[] {px, 0.0, AXLE_Z_IN}, new double[] {1.0, 0.0, 0.0},
                shapes, cells, RESTITUTION, FRICTION);
        return body.withHinge(new HingeDynamics(HIVE_MASS, new double[] {0.0, 0.0, COM_HEIGHT_IN}, INERTIA,
                DAMPING_PER_S, -TILT_RAD, TILT_RAD, 0.0));
    }

    /** Body angle that puts the given cell up. */
    public static double angleWithUp(boolean north) {
        return north ? TILT_RAD : -TILT_RAD;
    }
}
