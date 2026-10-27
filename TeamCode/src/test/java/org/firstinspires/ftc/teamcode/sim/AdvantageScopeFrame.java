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
 * AdvantageScope's FTC default, "Center/Rotated": origin at the field centre, metres, +X to the
 * right as seen from the RED alliance wall (RED's wall is our west wall, x = -72, where its
 * loading zone is), so their +X is our -Y and their +Y is our +X; headings turn by -90 deg.
 */
public final class AdvantageScopeFrame {

    static final double M_PER_IN = 0.0254;

    private AdvantageScopeFrame() {
    }

    public static Pose2d pose2d(double xIn, double yIn, double headingRad) {
        return new Pose2d(-yIn * M_PER_IN, xIn * M_PER_IN, new Rotation2d(headingRad - Math.PI / 2.0));
    }

    public static Translation3d point(double xIn, double yIn, double zIn) {
        return new Translation3d(-yIn * M_PER_IN, xIn * M_PER_IN, zIn * M_PER_IN);
    }

    public static Pose3d piece(double xIn, double yIn, double zIn) {
        return new Pose3d(point(xIn, yIn, zIn), new Rotation3d(0, 0, 0));
    }
}
