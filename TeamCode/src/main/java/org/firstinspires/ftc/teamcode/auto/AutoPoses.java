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
    public static final Pose START = new Pose(-13.0, -64.2, Math.PI / 2);
    public static final Pose SCORE_SOUTH = new Pose(-13.0, -36.0, Math.PI / 2);
    public static final Pose SCORE_NORTH = new Pose(-13.0, 44.0, -Math.PI / 2);
    public static final Pose FLOWER_NORTH_APPROACH = new Pose(-24.0, 42.0, Math.PI / 2);
    public static final Pose FLOWER_NORTH_PICKUP = new Pose(-24.0, 52.4, Math.PI / 2);
    public static final Pose ABOVE_GARDEN_LANE = new Pose(-36.0, -56.0, Math.PI);
    public static final Pose GARDEN_SWEEP_START = new Pose(-40.0, -62.5, Math.PI);
    public static final Pose GARDEN_SWEEP_END = new Pose(-56.0, -62.85, Math.PI);
    public static final Pose OFF_GARDEN_LANE = new Pose(-50.0, -50.0, Math.PI);

    // ---- Bezier control points ----
    public static final Pose[] TO_OTHER_SIDE = {
            new Pose(-46.0, -50.0), new Pose(-50.0, 4.79), new Pose(-40.0, 34.0)};
    public static final Pose[] TO_SCORE_NORTH = {new Pose(-24.0, 46.0), new Pose(-13.0, 60.0)};
    public static final Pose[] TO_GARDEN = {
            new Pose(-48.0, 50.0), new Pose(-50.0, -30.0), new Pose(-26.0, -56.0)};
    public static final Pose[] DRIVE_BACK = {
            new Pose(-50.0, -40.0), new Pose(-58.0, 62.0), new Pose(-13.0, 56.0)};

    // ---- headings along the curves, piece by piece (radians, RED), as in the draft ----
    public static final HeadingPiece[] TO_OTHER_SIDE_HEADING = {
            HeadingPiece.linear(0.2038, 1.5708, 2.3722), HeadingPiece.tangent(0.8679),
            HeadingPiece.linear(1.0, 1.0385, 1.5708)};
    public static final HeadingPiece[] TO_SCORE_NORTH_HEADING = {
            HeadingPiece.tangentReversed(0.2041), HeadingPiece.linear(0.5442, 2.8274, 4.7124),
            HeadingPiece.constant(1.0, -1.5708)};
    public static final HeadingPiece[] TO_GARDEN_HEADING = {
            HeadingPiece.linear(0.2186, -1.5708, -2.0354), HeadingPiece.tangent(1.0)};
    public static final HeadingPiece[] DRIVE_BACK_HEADING = {
            HeadingPiece.linear(0.1612, -3.1416, -1.7852), HeadingPiece.tangentReversed(0.707),
            HeadingPiece.linear(0.8242, -2.0008, -1.5708), HeadingPiece.constant(1.0, -1.5708)};

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
