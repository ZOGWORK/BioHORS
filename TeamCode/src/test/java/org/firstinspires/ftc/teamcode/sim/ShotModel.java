package org.firstinspires.ftc.teamcode.sim;

/**
 * Where and how fast a launched ball leaves the robot, from the robot's CAD (docs/ROBOT_CAD.md).
 *
 * Each shooter pinches the ball between its flywheel (above) and a fixed ramp (below), so the ball
 * rolls out along the ramp's top edge. A ball rolling between a moving rim and a fixed surface moves
 * at about half the rim speed; GRIP_EFFICIENCY is that fraction (PLACEHOLDER: real slip, ball
 * squish and the flywheel's speed dip per shot all lower it; measure it by filming a shot).
 *
 * Robot frame: forward, left, up from the floor, about the drive centre.
 */
public final class ShotModel {

    // MEASURED FROM CAD: the 72 mm Rhino wheel, 1:1 on the motor (team-confirmed).
    public static final double WHEEL_RADIUS_IN = 72.0 / 25.4 / 2.0;
    // PLACEHOLDER (physics of a ball rolling on a fixed hood: half the rim speed). NEEDS MEASUREMENT.
    public static double GRIP_EFFICIENCY = 0.5;

    // Pollen (right side). MEASURED FROM CAD: the top edge of the PS ramp (CAD x 2.56, z 9.6) plus
    // the ball's 1.4 in radius; the flywheel's centre plane 2.9 in right of the drive centre.
    public static final double POLLEN_FORWARD_IN = -0.9;
    public static final double POLLEN_LEFT_IN = -2.9;
    public static final double POLLEN_UP_IN = 12.1;
    // MEASURED FROM CAD: the PS ramp's slope at its top edge (2.3 in rise over 2.4 in).
    public static double POLLEN_PITCH_DEG = 44.0;

    // Nectar (left side). MEASURED FROM CAD: the nectar hood's top edge (CAD x 2.3, z 9.4).
    public static final double NECTAR_FORWARD_IN = -0.6;
    public static final double NECTAR_LEFT_IN = 2.6;
    public static final double NECTAR_UP_IN = 11.9;
    public static double NECTAR_PITCH_DEG = 49.0;

    private ShotModel() {
    }

    /** Ball exit speed, in/s, for a flywheel at the given rpm. */
    public static double exitSpeed(double rpm) {
        return rpm * 2.0 * Math.PI / 60.0 * WHEEL_RADIUS_IN * GRIP_EFFICIENCY;
    }

    /**
     * Field position {x, y, z} and velocity {vx, vy, vz} of a pollen leaving a robot at (x, y, h)
     * with the pollen flywheel at rpm (the robot's own motion is added in).
     */
    public static double[] pollenLaunch(double x, double y, double h, double rpm, double robotVx, double robotVy) {
        return launch(x, y, h, rpm, POLLEN_FORWARD_IN, POLLEN_LEFT_IN, POLLEN_UP_IN, POLLEN_PITCH_DEG,
                robotVx, robotVy);
    }

    static double[] launch(double x, double y, double h, double rpm, double fwd, double left, double up,
                           double pitchDeg, double robotVx, double robotVy) {
        double c = Math.cos(h), s = Math.sin(h);
        double v = exitSpeed(rpm), p = Math.toRadians(pitchDeg);
        return new double[] {
                x + fwd * c - left * s, y + fwd * s + left * c, up,
                v * Math.cos(p) * c + robotVx, v * Math.cos(p) * s + robotVy, v * Math.sin(p)};
    }
}
