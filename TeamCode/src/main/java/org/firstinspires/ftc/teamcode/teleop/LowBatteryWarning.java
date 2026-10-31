package org.firstinspires.ftc.teamcode.teleop;

/**
 * Warns the driver when the battery runs low, before shots start falling short. The voltage sags
 * for a moment under every spin-up, so it must stay below LOW_V for HOLD_S to warn, and above it
 * for HOLD_S to clear. Pure logic: the teleop passes the volts and the loop's dt.
 */
public class LowBatteryWarning {

    /** Below this, volts, the battery is low. SET BY HAND: a match battery starts near 12.8 V and a
     *  tired one rests near 11.5 V. NEEDS MEASUREMENT: the voltage at which shots fall short. */
    public static double LOW_V = 11.5;
    /** The voltage must stay on one side of LOW_V this long to change the warning, seconds. SET BY HAND. */
    public static double HOLD_S = 1.0;

    private boolean low;
    private double heldS;
    private double lastV = Double.NaN;

    /** Once per loop with the battery voltage (NaN if there is no sensor) and the loop's dt. */
    public void update(double volts, double dtS) {
        if (!Double.isFinite(volts) || volts <= 0 || !(dtS > 0)) return;
        lastV = volts;
        boolean belowNow = volts < LOW_V;
        if (belowNow == low) {
            heldS = 0;
            return;
        }
        heldS += dtS;
        if (heldS >= HOLD_S) {
            low = belowNow;
            heldS = 0;
        }
    }

    public boolean low() {
        return low;
    }

    /** The last voltage read, or NaN. */
    public double volts() {
        return lastV;
    }
}
