package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.aim.Alliance;

/**
 * Every point of the two-tip route, for RED, in the field frame (inches, radians, origin at the
 * field centre, +X toward BLUE, +Y away from the audience, CCW headings). The route was drafted in
 * Zenith (autos/two-tip.auto.json and autos/waypoints.json, which carry each point's provenance)
 * and copied here by hand. {@code AutoPosesTest} fails if the two drift apart: change the draft in
 * Zenith first, re-validate it, then copy the number here.
 *
 * BLUE is the point mirror of RED: (x, y, h) becomes (-x, -y, h + pi). {@link #of} does that.
 */
public final class AutoPoses {

    private AutoPoses() {
    }

    // ---- named waypoints (autos/waypoints.json) ----
    public static final Pose START = new Pose(-15.0, -64.2, Math.toRadians(90));
    public static final Pose SCORE_SOUTH = new Pose(-15.0, -36.0, 1.4868);
    public static final Pose GARDEN_APPROACH = new Pose(-62.5, -45.0, -Math.PI / 2);
    public static final Pose GARDEN_PICKUP = new Pose(-62.85, -57.0, -Math.PI / 2);
    public static final Pose GARDEN_TURN = new Pose(-50.0, -44.0, -Math.PI / 2);
    public static final Pose FLOWER_WEST_APPROACH = new Pose(-44.0, -24.0, Math.PI);
    public static final Pose FLOWER_WEST_PICKUP = new Pose(-52.4, -24.0, Math.PI);
    public static final Pose SCORE_NORTH = new Pose(-24.0, 44.0, -1.2576);
    public static final Pose FLOWER_NORTH_PICKUP = new Pose(-24.0, 52.4, Math.PI / 2);
    public static final Pose PARK = new Pose(-55.0, 36.0, Math.PI);

    // ---- turn-on-the-spot points: about an inch along the next heading (the path needs a length) ----
    public static final Pose FACE_SOUTH_CELL = new Pose(-14.96, -35.5);
    public static final Pose TURN_TO_GARDEN = new Pose(-15.99, -36.11);
    public static final Pose TURN_SOUTH_AT_GARDEN = new Pose(-52.0, -41.0);
    public static final Pose TURN_TO_SCORE_SOUTH = new Pose(-49.04, -43.71);
    public static final Pose TURN_TO_FLOWER_WEST = new Pose(-15.99, -36.12);
    public static final Pose TURN_NORTH = new Pose(-44.0, -23.0);
    public static final Pose NORTH_OF_SCORE_NORTH = new Pose(-24.0, 45.0);
    public static final Pose TURN_TO_PARK = new Pose(-24.97, 43.75);

    // ---- other path points ----
    public static final Pose GARDEN_LANE = new Pose(-52.0, -40.0);          // end of the leg west to the garden
    public static final Pose TO_SCORE_SOUTH_C1 = new Pose(-30.0, -38.0);    // Bezier control points
    public static final Pose TO_SCORE_SOUTH_C2 = new Pose(-15.0, -50.0);
    public static final Pose TO_FLOWER_WEST_C1 = new Pose(-32.0, -38.0);
    public static final Pose TO_FLOWER_WEST_C2 = new Pose(-38.0, -24.0);
    public static final Pose AROUND_WEST_C1 = new Pose(-46.0, 22.0);
    public static final Pose AROUND_WEST_C2 = new Pose(-27.08, 53.51);

    // ---- headings the turns go between (radians, RED) ----
    public static final double FACING_NORTH = Math.PI / 2;
    public static final double FACING_SOUTH = -Math.PI / 2;
    public static final double FACING_WEST = Math.PI;
    public static final double FACING_SOUTH_CELL = 1.4868;
    public static final double FACING_NORTH_CELL = -1.2576;
    public static final double TOWARD_GARDEN = -3.0339;
    public static final double TOWARD_SCORE_SOUTH = 0.2915;
    public static final double TOWARD_FLOWER_WEST = -3.0245;
    public static final double TOWARD_PARK = -2.889;

    /** A RED pose as seen by the given alliance. */
    public static Pose of(Alliance alliance, Pose red) {
        if (alliance == Alliance.RED) return red;
        return new Pose(-red.x(), -red.y(), red.heading() + Math.PI);
    }

    /** A RED heading as seen by the given alliance. */
    public static double heading(Alliance alliance, double red) {
        return alliance == Alliance.RED ? red : red + Math.PI;
    }
}
