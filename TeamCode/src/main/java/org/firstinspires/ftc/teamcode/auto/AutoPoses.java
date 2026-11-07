package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.aim.Alliance;

/**
 * Every point of the auto's route, for RED, in the field frame (inches, radians, origin at the
 * field centre, +X toward BLUE, +Y away from the audience, CCW headings). The route is Zenith's
 * starter example (examples/starter collect-and-score), tuned to this robot in Zenith
 * (autos/two-tip.auto.json and autos/waypoints.json, which carry each point's provenance), and
 * copied here by hand. {@code AutoPosesTest} fails if the two drift apart: change the draft in
 * Zenith first, re-validate it, then copy the number here.
 *
 * BLUE is the point mirror of RED: (x, y, h) becomes (-x, -y, h + pi). {@link #of} does that.
 */
public final class AutoPoses {

    private AutoPoses() {
    }

    // ---- named waypoints (autos/waypoints.json) ----
    public static final Pose START = new Pose(-13.0, -62.87, Math.PI / 2);
    public static final Pose SCORE_SOUTH = new Pose(-13.0, -50.0, Math.PI / 2);   // the team, 2026-11-07: 2 in further back (was -48)
    public static final Pose SCORE_NORTH = new Pose(-13.0, 48.0, -Math.PI / 2);
    /** The second volley there (the garden pollen): 3 in further back from the hive (the team, 2026-11-07). */
    public static final Pose SCORE_NORTH_SECOND = new Pose(-13.0, 51.0, -Math.PI / 2);
    public static final Pose FLOWER_NORTH_APPROACH = new Pose(-21.0, 42.0, Math.PI / 2);
    public static final Pose FLOWER_NORTH_PICKUP = new Pose(-21.0, 58.0, Math.PI / 2);
    public static final Pose ABOVE_GARDEN_LANE = new Pose(-36.0, -54.7, Math.PI);
    public static final Pose GARDEN_SWEEP_START = new Pose(-40.0, -61.4, Math.PI);
    public static final Pose GARDEN_SWEEP_END = new Pose(-58.5, -61.62, Math.PI);
    public static final Pose OFF_GARDEN_LANE = new Pose(-50.0, -50.0, Math.PI);
    public static final Pose PARK_LANE = new Pose(-61.5, 54.0, 0.0);
    public static final Pose PARK = new Pose(-61.5, 49.0, 0.0);

    // ---- Bezier control points ----
    public static final Pose[] TO_OTHER_SIDE = {
            new Pose(-68.0, -31.5), new Pose(-50.6, -3.0), new Pose(-46.8, 19.5)};
    public static final Pose[] TO_SCORE_NORTH = {new Pose(-21.0, 46.0), new Pose(-13.0, 60.0)};
    public static final Pose[] TO_GARDEN = {
            new Pose(-55.2, 30.3), new Pose(-58.7, -27.9), new Pose(-26.0, -54.7)};
    public static final Pose[] DRIVE_BACK = {
            new Pose(-50.3, -5.3), new Pose(-57.6, 16.1), new Pose(-13.0, 56.0)};
    public static final Pose[] TO_PARK_LANE = {new Pose(-13.0, 55.0), new Pose(-35.0, 55.0)};

    // ---- headings along the curves, piece by piece (radians, RED), as in the draft ----
    // Written by tools/zenith_headings.py: Pedro reads a piece's end as a fraction of the
    // path's length and hands a tangent piece its own 0..1 t, so the starter's tangent spans are
    // linear pieces between the curve's true direction, each starting where the last ended.
    // AutoPosesTest checks there is no jump.
    public static final HeadingPiece[] TO_OTHER_SIDE_HEADING = {
            HeadingPiece.linear(0.2038, 1.5708, 2.5276), HeadingPiece.linear(0.3038, 2.5276, 2.2384),
            HeadingPiece.linear(0.4038, 2.2384, 1.8396), HeadingPiece.linear(0.5038, 1.8396, 1.4854),
            HeadingPiece.linear(0.6038, 1.4854, 1.2526), HeadingPiece.linear(0.7038, 1.2526, 1.0923),
            HeadingPiece.linear(0.8038, 1.0923, 0.9581), HeadingPiece.linear(0.9, 0.9581, 0.8359),
            HeadingPiece.linear(1.0, 0.8359, 1.5708)};
    public static final HeadingPiece[] TO_SCORE_NORTH_HEADING = {
            HeadingPiece.constant(0.2041, 1.5708), HeadingPiece.linear(0.3742, 1.5708, 3.1416),
            HeadingPiece.linear(0.5442, 3.1416, 4.7124), HeadingPiece.constant(1.0, -1.5708)};
    public static final HeadingPiece[] TO_GARDEN_HEADING = {
            HeadingPiece.linear(0.1, -1.5708, -2.5143), HeadingPiece.linear(0.2, -2.5143, -2.2812),
            HeadingPiece.linear(0.3, -2.2812, -2.0604), HeadingPiece.linear(0.4, -2.0604, -1.8605),
            HeadingPiece.linear(0.5, -1.8605, -1.6829), HeadingPiece.linear(0.6, -1.6829, -1.525),
            HeadingPiece.linear(0.7, -1.525, -1.3838), HeadingPiece.linear(0.8, -1.3838, -1.259),
            HeadingPiece.linear(0.9, -1.259, -1.1682), HeadingPiece.linear(1.0, -1.1682, -3.1416)};
    public static final HeadingPiece[] DRIVE_BACK_HEADING = {
            HeadingPiece.linear(0.1612, 3.1416, 4.7296), HeadingPiece.linear(0.2612, 4.7296, 4.6838),
            HeadingPiece.linear(0.3612, 4.6838, 4.5914), HeadingPiece.linear(0.4612, 4.5914, 4.4626),
            HeadingPiece.linear(0.5612, 4.4626, 4.321), HeadingPiece.linear(0.6612, 4.321, 4.1877),
            HeadingPiece.linear(0.7612, 4.1877, 4.0695), HeadingPiece.linear(0.86, 4.0695, 3.9587),
            HeadingPiece.linear(0.97, 3.9587, 4.7124), HeadingPiece.constant(1.0, -1.5708)};

    /** One piece of a path's heading, up to {@code endT} of the path: Zenith's piecewise modes. */
    public static final class HeadingPiece {
        public final double endT;
        public final String mode;     // "linear", "tangent", "tangentReversed" or "constant"
        public final double from;     // linear: start heading; constant: the heading
        public final double to;       // linear: end heading

        private HeadingPiece(double endT, String mode, double from, double to) {
            this.endT = endT;
            this.mode = mode;
            this.from = from;
            this.to = to;
        }

        public static HeadingPiece linear(double endT, double from, double to) {
            return new HeadingPiece(endT, "linear", from, to);
        }

        public static HeadingPiece tangent(double endT) {
            return new HeadingPiece(endT, "tangent", Double.NaN, Double.NaN);
        }

        public static HeadingPiece tangentReversed(double endT) {
            return new HeadingPiece(endT, "tangentReversed", Double.NaN, Double.NaN);
        }

        public static HeadingPiece constant(double endT, double heading) {
            return new HeadingPiece(endT, "constant", heading, Double.NaN);
        }
    }

    // ---- headings (radians, RED) ----
    public static final double FACING_NORTH = Math.PI / 2;
    public static final double FACING_SOUTH = -Math.PI / 2;
    public static final double FACING_WEST = Math.PI;
    public static final double FACING_EAST = 0.0;

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
