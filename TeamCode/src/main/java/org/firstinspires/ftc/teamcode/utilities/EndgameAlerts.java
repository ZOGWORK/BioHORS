package org.firstinspires.ftc.teamcode.utilities;

/**
 * When to rumble the drivers during the teleop: once as the endgame starts (nectar time), once more
 * FINAL_WARNING_S before the end. Pure logic: the teleop passes the seconds since PLAY.
 */
public class EndgameAlerts {

    /** The driver-controlled period, seconds. CARRIED OVER: FTC's usual 2:00; check the BIOBUZZ manual. */
    public static double TELEOP_S = 120.0;
    /** The endgame, the last this many seconds. CARRIED OVER: FTC's usual 30 s; check the BIOBUZZ manual. */
    public static double ENDGAME_S = 30.0;
    /** The last warning, this many seconds before the end. SET BY HAND. */
    public static double FINAL_WARNING_S = 10.0;

    public enum Cue { NONE, ENDGAME, FINAL_WARNING }

    private boolean endgameGiven;
    private boolean finalGiven;

    /** Once per loop. Returns the cue to give now (each one exactly once), else NONE. */
    public Cue update(double sincePlayS) {
        if (!Double.isFinite(sincePlayS)) return Cue.NONE;
        if (!finalGiven && sincePlayS >= TELEOP_S - FINAL_WARNING_S) {
            finalGiven = true;
            endgameGiven = true; // past both: only the later one is worth a rumble
            return Cue.FINAL_WARNING;
        }
        if (!endgameGiven && sincePlayS >= TELEOP_S - ENDGAME_S) {
            endgameGiven = true;
            return Cue.ENDGAME;
        }
        return Cue.NONE;
    }

    /** Seconds left in the teleop (negative once it is over). */
    public static double remainingS(double sincePlayS) {
        return TELEOP_S - sincePlayS;
    }
}
