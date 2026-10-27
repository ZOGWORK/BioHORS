package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.field.GamePieceTracker;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.util.HashSet;
import java.util.Locale;
import java.util.Set;

/**
 * Where in the up cell to aim a rapid volley: its middle, a side wall, or a corner. Fires 8 real
 * balls (ShotModel, under ZenithAutoMatchTest's WHAT-IF shot) into RED's up (south) cell of the
 * physics hive and counts how many seat.
 *
 * Aim points are in the cell's plan footprint: x across the 20 in opening (0 = its middle, +/-8 =
 * a ball's width off a side wall), y from the front lip (18.58 in south of the axle) to the deep end
 * (8.16 in, the inner wall at the 30 deg stop). How deep a ball lands is set by the rpm.
 *
 * Finding: aiming at the cell's middle seats as many or more than any wall or corner, with or
 * without shot scatter and the flywheel's dip between rapid shots, and for dead (0.02) to lively
 * (0.5) hive walls. The up cell slopes down to its inner wall, so every ball that gets in rolls to
 * the same place; aiming off-centre only adds the chance of clipping the lip or a side wall.
 *
 * The full printed sweeps take about a minute: run them with -Dsim.sweeps=true.
 */
class PocketAimTest {

    static final double HIVE_X = -12.75;
    static final double SHOOTER_RESPONSE_PER_S = 3.0; // as BioHorsSim (PLACEHOLDER there)
    static final double LIP_Y = -18.58, MID_Y = -13.37, DEEP_Y = -8.16;

    /** Heading from (rx, ry) whose pollen track (offset exit) passes over (tx, ty). */
    static double headingThrough(double rx, double ry, double tx, double ty) {
        double h = Math.atan2(ty - ry, tx - rx);
        for (int i = 0; i < 4; i++) {
            double[] l = ShotModel.pollenLaunch(rx, ry, h, 0, 0, 0);
            h = Math.atan2(ty - l[1], tx - l[0]);
        }
        return h;
    }

    /** Pollen seated (ever) out of n, fired every cadence seconds at (tx, ty) from (rx, ry). */
    static int volley(double rx, double ry, double tx, double ty, double rpm, int n, double cadence) {
        return volley(rx, ry, tx, ty, rpm, n, cadence, 0, 0, 0, 0);
    }

    /**
     * As above, with real rapid fire: each shot scatters (seeded, normal: heading sigma in deg,
     * speed sigma as a fraction) and pulls the flywheel down by dip (a fraction of its speed), which
     * recovers toward the target at the sim's shooter response between shots.
     */
    static int volley(double rx, double ry, double tx, double ty, double rpm, int n, double cadence,
                      double headingSigmaDeg, double speedSigma, double dip, long seed) {
        java.util.Random rnd = new java.util.Random(seed);
        double wheel = rpm;
        GamePieceTracker tracker = BiobuzzHiveTest.tracker(BiobuzzHive.body(true));
        String hive = BiobuzzHive.bodyName(true);
        tracker.setBodyMotion(hive, BiobuzzHive.angleWithUp(false), 0.0);
        double h = headingThrough(rx, ry, tx, ty);
        Set<Integer> ids = new HashSet<>(), seated = new HashSet<>();
        double t = 0, next = 0;
        int fired = 0;
        while (t < n * cadence + 4.0) {
            if (fired < n && t >= next - 1e-9) {
                double hShot = h + Math.toRadians(headingSigmaDeg * rnd.nextGaussian());
                double[] l = ShotModel.pollenLaunch(rx, ry, hShot, wheel * (1 + speedSigma * rnd.nextGaussian()), 0, 0);
                wheel *= 1 - dip;
                int id = tracker.addHeldPiece("pollen", BiobuzzHiveTest.R);
                tracker.launch(id, "shot", l[0], l[1], l[2], l[3], l[4], l[5]);
                ids.add(id);
                fired++;
                next += cadence;
            }
            tracker.advance(0.02);
            t += 0.02;
            wheel = rpm - (rpm - wheel) * Math.exp(-SHOOTER_RESPONSE_PER_S * 0.02);
            for (int id : ids) {
                String cell = tracker.getPiece(id).getSeatedCellName();
                if (cell != null && cell.startsWith(hive)) seated.add(id);
            }
        }
        return seated.size();
    }

    /** Lowest rpm (50 rpm steps) at which one ball aimed at the cell's middle seats, or -1. */
    static int minRpmFrom(double rx, double ry) {
        for (int rpm = 1000; rpm <= 6000; rpm += 50) {
            if (volley(rx, ry, HIVE_X, MID_Y, rpm, 1, 1.0) == 1) return rpm;
        }
        return -1;
    }

    static final double[][] AIMS = {
            {0, MID_Y}, {0, LIP_Y}, {0, DEEP_Y},
            {-8, MID_Y}, {8, MID_Y}, {-6, MID_Y}, {6, MID_Y},
            {-8, LIP_Y}, {8, LIP_Y}, {-8, DEEP_Y}, {8, DEEP_Y}};

    static void sweepsOnly() {
        Assumptions.assumeTrue(Boolean.getBoolean("sim.sweeps"), "printed sweep: run with -Dsim.sweeps=true");
    }

    /** The finding above, kept cheap: the worst scatter case from the centre line, 5 seeds. */
    @Test
    void aimingAtTheMiddleSeatsAtLeastAsManyAsAWallOrCorner() {
        ShotSweepTest.withShot(ZenithAutoMatchTest.WHAT_IF_PITCH_DEG, ZenithAutoMatchTest.WHAT_IF_GRIP, () -> {
            double[] spot = {HIVE_X, -50};
            int base = minRpmFrom(spot[0], spot[1]);
            double middle = bestMean(spot, new double[] {0, MID_Y}, base);
            assertTrue(middle >= 6.0, "aimed at the middle, only " + middle + " of 8 seat");
            for (double[] aim : new double[][] {{-8, MID_Y}, {8, MID_Y}, {-8, LIP_Y}, {8, LIP_Y}, {-8, DEEP_Y}, {8, DEEP_Y}}) {
                double m = bestMean(spot, aim, base);
                assertTrue(m <= middle + 0.2, String.format(Locale.ROOT,
                        "aiming at (%+.0f, %.1f) seats %.1f of 8, the middle %.1f: the pocket wins", aim[0], aim[1], m, middle));
            }
        });
    }

    /** Best mean seated of 8 over rpm base+100 and +200, 5 seeds, 3 deg / 3 % scatter, 8 % dip. */
    static double bestMean(double[] spot, double[] aim, int base) {
        double best = 0;
        for (int a = 100; a <= 200; a += 100) {
            int total = 0;
            for (long seed = 0; seed < 5; seed++) {
                total += volley(spot[0], spot[1], HIVE_X + aim[0], aim[1], base + a, 8, 0.30, 3.0, 0.03, 0.08, seed);
            }
            best = Math.max(best, total / 5.0);
        }
        return best;
    }

    @Test
    void volleysByAimPointArePrinted() {
        sweepsOnly();
        ShotSweepTest.withShot(ZenithAutoMatchTest.WHAT_IF_PITCH_DEG, ZenithAutoMatchTest.WHAT_IF_GRIP, () -> {
            double[][] spots = {{HIVE_X, -50}, {-30, -48}};
            for (double[] s : spots) {
                int base = minRpmFrom(s[0], s[1]);
                for (double cadence : new double[] {0.30, 0.15}) {
                    StringBuilder head = new StringBuilder(String.format(Locale.ROOT,
                            "POCKET spot (%.1f, %.0f) base %d rpm, every %.2f s | aim dx,y  rpm+", s[0], s[1], base, cadence));
                    int[] adds = {0, 100, 200, 300, 400};
                    for (int a : adds) head.append(String.format(Locale.ROOT, "%5d", a));
                    System.out.println(head);
                    for (double[] aim : AIMS) {
                        StringBuilder row = new StringBuilder(String.format(Locale.ROOT,
                                "POCKET   %+5.0f,%6.1f                                          ", aim[0], aim[1]));
                        int total = 0;
                        for (int a : adds) {
                            int k = volley(s[0], s[1], HIVE_X + aim[0], aim[1], base + a, 8, cadence);
                            total += k;
                            row.append(String.format(Locale.ROOT, "%5d", k));
                        }
                        System.out.println(row.append(String.format(Locale.ROOT, "  sum %d", total)));
                    }
                }
            }
        });
    }

    /** Mean seated out of 8 over 10 seeds, with scatter and the per-shot dip. */
    static double meanSeated(double[] spot, double[] aim, double rpm, double cadence, double hSig, double vSig, double dip) {
        int total = 0;
        for (long seed = 0; seed < 10; seed++) {
            total += volley(spot[0], spot[1], HIVE_X + aim[0], aim[1], rpm, 8, cadence, hSig, vSig, dip, seed);
        }
        return total / 10.0;
    }

    @Test
    void realRapidFireByAimPointIsPrinted() {
        sweepsOnly();
        ShotSweepTest.withShot(ZenithAutoMatchTest.WHAT_IF_PITCH_DEG, ZenithAutoMatchTest.WHAT_IF_GRIP, () -> {
            double[][] spots = {{HIVE_X, -50}, {-30, -48}};
            // scatter cases: {heading sigma deg, speed sigma, dip per shot}
            double[][] cases = {{1.5, 0.02, 0.05}, {3.0, 0.03, 0.08}};
            for (double[] s : spots) {
                int base = minRpmFrom(s[0], s[1]);
                for (double[] c : cases) {
                    StringBuilder head = new StringBuilder(String.format(Locale.ROOT,
                            "REAL spot (%.1f, %.0f) base %d, every 0.30 s, scatter %.1f deg / %.0f %%, dip %.0f %% | rpm+",
                            s[0], s[1], base, c[0], 100 * c[1], 100 * c[2]));
                    int[] adds = {0, 100, 200, 300};
                    for (int a : adds) head.append(String.format(Locale.ROOT, "%6d", a));
                    System.out.println(head);
                    for (double[] aim : AIMS) {
                        StringBuilder row = new StringBuilder(String.format(Locale.ROOT, "REAL   aim %+3.0f,%6.1f   ", aim[0], aim[1]));
                        double best = 0;
                        for (int a : adds) {
                            double m = meanSeated(s, aim, base + a, 0.30, c[0], c[1], c[2]);
                            best = Math.max(best, m);
                            row.append(String.format(Locale.ROOT, "%6.1f", m));
                        }
                        System.out.println(row.append(String.format(Locale.ROOT, "   best %.1f", best)));
                    }
                }
            }
        });
    }

    @Test
    void deadAndLivelyHiveWallsArePrinted() {
        sweepsOnly();
        ShotSweepTest.withShot(ZenithAutoMatchTest.WHAT_IF_PITCH_DEG, ZenithAutoMatchTest.WHAT_IF_GRIP, () -> {
            double[][] spots = {{HIVE_X, -50}, {-30, -48}};
            double savedE = BiobuzzHive.RESTITUTION;
            try {
                for (double e : new double[] {0.02, 0.5}) {
                    BiobuzzHive.RESTITUTION = e;
                    for (double[] s : spots) {
                        int base = minRpmFrom(s[0], s[1]);
                        System.out.println(String.format(Locale.ROOT,
                                "WALLS e %.2f spot (%.1f, %.0f) base %d, scatter 3 deg / 3 %%, dip 8 %% | best of rpm+0..300",
                                e, s[0], s[1], base));
                        for (double[] aim : AIMS) {
                            double best = 0;
                            for (int a = 0; a <= 300; a += 100) {
                                best = Math.max(best, meanSeated(s, aim, base + a, 0.30, 3.0, 0.03, 0.08));
                            }
                            System.out.println(String.format(Locale.ROOT, "WALLS   aim %+3.0f,%6.1f   %.1f", aim[0], aim[1], best));
                        }
                    }
                }
            } finally {
                BiobuzzHive.RESTITUTION = savedE;
            }
        });
    }
}
