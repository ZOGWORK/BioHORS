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
    public static final Pose SCORE_SOUTH = new Pose(-13.0, -48.0, Math.PI / 2);
    public static final Pose SCORE_NORTH = new Pose(-13.0, 48.0, -Math.PI / 2);
    public static final Pose FLOWER_NORTH_APPROACH = new Pose(-24.0, 42.0, Math.PI / 2);
    public static final Pose FLOWER_NORTH_PICKUP = new Pose(-24.0, 52.85, Math.PI / 2);
    public static final Pose ABOVE_GARDEN_LANE = new Pose(-36.0, -54.7, Math.PI);
    public static final Pose GARDEN_SWEEP_START = new Pose(-40.0, -61.17, Math.PI);
    public static final Pose GARDEN_SWEEP_END = new Pose(-59.6, -61.62, Math.PI);
    public static final Pose OFF_GARDEN_LANE = new Pose(-50.0, -50.0, Math.PI);
    public static final Pose PARK_LANE = new Pose(-54.7, 54.0, Math.PI);
    public static final Pose PARK = new Pose(-54.7, 36.0, Math.PI);

    // ---- Bezier control points ----
    public static final Pose[] TO_OTHER_SIDE = {
            new Pose(-46.0, -50.0), new Pose(-50.0, 4.79), new Pose(-40.0, 34.0)};
    public static final Pose[] TO_SCORE_NORTH = {new Pose(-24.0, 46.0), new Pose(-13.0, 60.0)};
    public static final Pose[] TO_GARDEN = {
            new Pose(-48.0, 50.0), new Pose(-50.0, -30.0), new Pose(-26.0, -54.7)};
    public static final Pose[] DRIVE_BACK = {
            new Pose(-50.0, -40.0), new Pose(-58.0, 62.0), new Pose(-13.0, 56.0)};
    public static final Pose[] TO_PARK_LANE = {new Pose(-13.0, 55.0), new Pose(-35.0, 55.0)};

    // ---- headings along the curves, piece by piece (radians, RED), as in the draft ----
    // Written by tools/zenith_headings.py: Pedro reads a piece's end as a fraction of the
    // path's length and hands a tangent piece its own 0..1 t, so the starter's tangent spans are
    // linear pieces between the curve's true direction, each starting where the last ended.
    // AutoPosesTest checks there is no jump.
    public static final HeadingPiece[] TO_OTHER_SIDE_HEADING = {
            HeadingPiece.linear(0.2038, 1.5708, 2.2316), HeadingPiece.linear(0.3038, 2.2316, 1.9476),
            HeadingPiece.linear(0.4038, 1.9476, 1.7628), HeadingPiece.linear(0.5038, 1.7628, 1.6217),
            HeadingPiece.linear(0.6038, 1.6217, 1.4946), HeadingPiece.linear(0.7038, 1.4946, 1.3587),
            HeadingPiece.linear(0.8038, 1.3587, 1.1845), HeadingPiece.linear(0.8679, 1.1845, 1.0273),
            HeadingPiece.linear(1.0, 1.0273, 1.5708)};
    public static final HeadingPiece[] TO_SCORE_NORTH_HEADING = {
            HeadingPiece.constant(0.2041, 1.5708), HeadingPiece.linear(0.3742, 1.5708, 3.1416),
            HeadingPiece.linear(0.5442, 3.1416, 4.7124), HeadingPiece.constant(1.0, -1.5708)};
    public static final HeadingPiece[] TO_GARDEN_HEADING = {
            HeadingPiece.linear(0.2186, -1.5708, -2.0558), HeadingPiece.linear(0.3186, -2.0558, -1.8395),
            HeadingPiece.linear(0.4186, -1.8395, -1.7005), HeadingPiece.linear(0.5186, -1.7005, -1.5969),
            HeadingPiece.linear(0.6186, -1.5969, -1.5113), HeadingPiece.linear(0.7186, -1.5113, -1.4357),
            HeadingPiece.linear(0.8186, -1.4357, -1.3693), HeadingPiece.linear(0.9186, -1.3693, -1.348),
            HeadingPiece.linear(1.0, -1.348, -3.1416)};
    public static final HeadingPiece[] DRIVE_BACK_HEADING = {
            HeadingPiece.linear(0.1612, 3.1416, 4.7009), HeadingPiece.linear(0.2612, 4.7009, 4.656),
            HeadingPiece.linear(0.3612, 4.656, 4.6031), HeadingPiece.linear(0.4612, 4.6031, 4.5376),
            HeadingPiece.linear(0.5612, 4.5376, 4.4518), HeadingPiece.linear(0.6612, 4.4518, 4.3299),
            HeadingPiece.linear(0.707, 4.3299, 4.2536), HeadingPiece.linear(0.8242, 4.2536, 4.7124),
            HeadingPiece.constant(1.0, -1.5708)};

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
