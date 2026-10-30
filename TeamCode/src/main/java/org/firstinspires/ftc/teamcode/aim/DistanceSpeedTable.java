package org.firstinspires.ftc.teamcode.aim;

/**
 * Flywheel speed by distance to the target: an ordered table of (distance, rpm) points, read with
 * straight-line interpolation between points and held at the end points outside the table.
 *
 * A one-point table is a constant. That is the right table before anything is measured: the
 * teleop seeds each shooter's table with its fixed target rpm, so nothing changes until the team
 * adds measured points (MechanismConstants).
 *
 * Fails soft: a table that is empty, has mismatched columns, a non-finite or negative entry, or
 * distances that do not strictly increase is unusable, and every read of it returns the caller's
 * fallback rpm. So does a non-finite distance. Pure math, no hardware.
 */
public final class DistanceSpeedTable {

    private final double[] distancesIn;
    private final double[] rpms;
    private final boolean usable;

    /**
     * @param distancesIn distances to the target, inches, strictly increasing
     * @param rpms        flywheel rpm at each distance
     */
    public DistanceSpeedTable(double[] distancesIn, double[] rpms) {
        boolean ok = distancesIn != null && rpms != null
                && distancesIn.length >= 1 && distancesIn.length == rpms.length;
        if (ok) {
            for (int i = 0; i < distancesIn.length; i++) {
                if (!Double.isFinite(distancesIn[i]) || !Double.isFinite(rpms[i]) || rpms[i] < 0.0) ok = false;
                if (i > 0 && !(distancesIn[i] > distancesIn[i - 1])) ok = false;
            }
        }
        usable = ok;
        this.distancesIn = ok ? distancesIn.clone() : new double[0];
        this.rpms = ok ? rpms.clone() : new double[0];
    }

    /** False when the table was malformed; every read then returns the fallback. */
    public boolean isUsable() {
        return usable;
    }

    /** Number of points (0 when unusable). */
    public int size() {
        return distancesIn.length;
    }

    /**
     * Flywheel rpm at {@code distanceIn}, or {@code fallbackRpm} when the distance is not finite or
     * the table is unusable.
     */
    public double rpmAt(double distanceIn, double fallbackRpm) {
        if (!usable || !Double.isFinite(distanceIn)) return fallbackRpm;
        int last = distancesIn.length - 1;
        if (distanceIn <= distancesIn[0]) return rpms[0];
        if (distanceIn >= distancesIn[last]) return rpms[last];
        int i = 1;
        while (distancesIn[i] < distanceIn) i++;
        double f = (distanceIn - distancesIn[i - 1]) / (distancesIn[i] - distancesIn[i - 1]);
        return rpms[i - 1] + f * (rpms[i] - rpms[i - 1]);
    }
}
