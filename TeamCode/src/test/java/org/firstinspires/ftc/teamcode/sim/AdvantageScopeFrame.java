package org.firstinspires.ftc.teamcode.sim;

import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

/**
 * Converts this project's field frame to AdvantageScope's, so RLOGs show the robot and the pollen
 * on AdvantageScope's "2026-2027 Field" (2D and 3D Field tabs).
 *
 * Ours (Zenith's field file, the autos, aim): origin at the field centre, inches, +X toward the
 * audience's right (BLUE side), +Y away from the audience, heading CCW from +X.
 *
 * AdvantageScope's 2026-2027 field model uses the same axes, in metres: no rotation. Checked
 * against the model itself (autoAssets/Field3d_20262027FTCFieldV1, AdvantageScopeFrameTest): its
 * staged pollen lie where the field file puts them with x = x and y = y (the red garden at
 * X -1.76..-1.54, Y -1.76; the flowers at (-1.73, -0.59), (-0.59, 1.73), (1.73, 0.59),
 * (0.59, -1.73)), its driver stations are at X = +/-2.5 (the alliance walls are our x = +/-72),
 * and its red hive's raised cell is the south one, as the field file says. An earlier version
 * rotated by 90 deg and drew everything a quarter turn off.
 */
public final class AdvantageScopeFrame {

    static final double M_PER_IN = 0.0254;

    private AdvantageScopeFrame() {
    }

    public static Pose2d pose2d(double xIn, double yIn, double headingRad) {
        return new Pose2d(xIn * M_PER_IN, yIn * M_PER_IN, new Rotation2d(headingRad));
    }

    public static Translation3d point(double xIn, double yIn, double zIn) {
        return new Translation3d(xIn * M_PER_IN, yIn * M_PER_IN, zIn * M_PER_IN);
    }

    public static Pose3d piece(double xIn, double yIn, double zIn) {
        return new Pose3d(point(xIn, yIn, zIn), new Rotation3d(0, 0, 0));
    }

    /**
     * A hive bar turned by angleRad about its axle (our +X, AdvantageScope's +X too: a roll), the
     * axle at (xIn, 0, zIn).
     */
    public static Pose3d hive(double xIn, double zIn, double angleRad) {
        return new Pose3d(point(xIn, 0, zIn), new Rotation3d(angleRad, 0, 0));
    }

    /**
     * A flywheel component of the custom robot model (advantagescope/Robot_BioHORS), relative to the
     * robot: its axle at (forwardIn, 0, upIn) in the robot frame (x forward, y left, z up, origin the
     * drive centre on the floor), turned spinRad about the robot's +y (left) axis.
     */
    public static Pose3d robotFlywheel(double forwardIn, double upIn, double spinRad) {
        return new Pose3d(new Translation3d(forwardIn * M_PER_IN, 0, upIn * M_PER_IN), new Rotation3d(0, spinRad, 0));
    }
}
