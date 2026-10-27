package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePiecePhysicsConfig;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.zenith.Json;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Pose3d;

import java.util.ArrayList;
import java.util.List;

/**
 * The BIOBUZZ field around the simulated robot during AUTO: where the pollen is, what the robot
 * really holds, what its launches really do, and the hive cells and tips. Built from the project's
 * own field file (autos/field/biobuzz.field.json), so a corrected measurement there moves it.
 *
 * It reads only the simulated WORLD (BioHorsSim: the plant's true pose, the motors' real power, the
 * gate servo's real position), never what the robot code believes.
 *
 * Model, stated plainly (every number labelled; none measured on a robot):
 *  - Intake: a pollen is taken in when it overlaps the intake mouth (robot.json mouths[0]) while
 *    the intake motors run inward and the robot holds fewer than 4.
 *  - Flower: a running intake whose mouth is at a flower's retrieval opening (the face of the
 *    flower's foot toward the field), facing it, WITH THE FLOWER BAR DOWN (its servo really at the
 *    down position), pulls one pollen from the bottom every FLOWER_S_PER_PIECE.
 *  - Launch: with the pollen gate really open, the intake feeding and the pollen flywheel at
 *    least LAUNCH_MIN_RPM, one pollen leaves every LAUNCH_S_PER_PIECE.
 *  - A launch scores in the alliance's up cell only from that cell's floor quarter, outboard of
 *    its outer face, facing the cell within LAUNCH_CONE_DEG. Otherwise it lands loose.
 *  - Tip: a cell tips when its pollen reaches the field file's tip table entry for the nectar in
 *    it (rules.tipTable; this project's copy says 8). The cell empties and the other comes up.
 *    The 4 s swing is not modelled, except that for TIP_CATCH_S after a tip a launch still goes into
 *    the cell that tipped (and spills with it); after that, shots go to the new up cell.
 *  - Physics mode (usePhysics): instead of the rule above, each launch is a real ball in SimLoop's
 *    physics, fired from the CAD shooter exit at the speed the flywheel's real rpm gives
 *    (ShotModel), into the physics hive (BiobuzzHive). A pollen scores when it first seats in one
 *    of the hive's cells; a tip is the hive swinging from one stop to the other under the load;
 *    the tipped cell's pollen roll out onto the floor. The staged nectar is not modelled there.
 *  - Intake load: each pollen aboard slows both intake motors by INTAKE_LOAD_PER_PIECE while they
 *    run inward, which is what the robot's intake-encoder pollen count reads.
 *  - Contact: the footprint (robot.json) against the field file's low obstacles (hive frame
 *    bars and legs, flower feet) and the walls.
 *
 * For AdvantageScope (2026-2027 Field, 2D or 3D Field tab) it logs the robot's true pose as
 * Field/Robot and every pollen as Field/Pollen: the garden line, the flower columns, the ones
 * aboard, each launch in flight (a 0.45 s arc), the ones in the hive cells, and a tipped cell's
 * spill on the floor (drawn only; the spill is not modelled as collectable). With the custom robot
 * model (advantagescope/Robot_BioHORS) it also logs Field/RobotComponents: the pollen and nectar
 * flywheels, spinning at their true rpm.
 */
public final class BiobuzzMatch {

    static final double FLOWER_S_PER_PIECE = 0.15;  // APPROX (DSim, per Zenith's BIOBUZZ ledger)
    static final double LAUNCH_S_PER_PIECE = 0.30;  // PLACEHOLDER: real launch cadence NEEDS MEASUREMENT
    static final double LAUNCH_MIN_RPM = 1440.0;    // PLACEHOLDER: 90 % of the 1600 rpm test target
    static final double LAUNCH_CONE_DEG = 15.0;     // PLACEHOLDER: how far off the cell a launch still goes in
    static final double OPENING_REACH_IN = 2.0;     // PLACEHOLDER: mouth to retrieval opening
    static final double FACING_CONE_DEG = 25.0;     // PLACEHOLDER
    static final double CONTACT_TOLERANCE_IN = 0.25;
    // MEASURED FROM CAD: each flywheel's 8 mm shaft centre, in the robot frame (docs/ROBOT_CAD.md).
    // advantagescope/Robot_BioHORS/config.json zeroes the components at these same points.
    static final double POLLEN_AXLE_FORWARD_IN = -5.3945, POLLEN_AXLE_UP_IN = 11.613;
    static final double NECTAR_AXLE_FORWARD_IN = -5.6115, NECTAR_AXLE_UP_IN = 11.5365;
    // APPROX: the ball rides under the wheel toward the front, so the wheel's underside moves
    // forward: positive rpm turns it negatively about the robot's left (+y) axis. Drawing only.
    static final double FLYWHEEL_SPIN_SIGN = -1.0;
    /** Load each pollen aboard puts on both intake motors (a resisting torque fraction).
     *  PLACEHOLDER: 0.07, so a full robot reads clearly below MAINHORS26's 2100 ticks/s loaded
     *  threshold (1960) as its loaded robot did, and 1-3 pollen still read as 1-3. */
    static final double INTAKE_LOAD_PER_PIECE = 0.07;
    /** For this long after a tip, a launch still goes into the cell that tipped (it has only just started
     *  to swing) and spills with it. PLACEHOLDER: the hive swings for about 4 s (DSim). */
    static final double TIP_CATCH_S = 0.5;
    static final double GATE_OPEN_THRESHOLD = (0.485 + 0.67) / 2.0; // halfway between MechanismConstants' positions

    private final BioHorsSim sim;
    private final boolean red;
    private final double robotLength, robotWidth, robotHeight;
    private final double corForward, corLeft; // where the turning centre sits in the box (robot.json)
    private final double mouthOffsetX, mouthOffsetY, mouthHalfWidth, mouthHalfDepth;
    private final int[] tipTable;

    private final List<double[]> loosePollen = new ArrayList<>();   // {x, y, r}
    private final List<Flower> flowers = new ArrayList<>();
    private final List<double[]> boxes = new ArrayList<>();         // {minX, maxX, minY, maxY}
    private final List<String> boxNames = new ArrayList<>();
    private final List<double[]> struts = new ArrayList<>();        // {x0,y0,z0,x1,y1,z1,r}
    private final List<String> strutNames = new ArrayList<>();

    private final double hiveX;
    private boolean upIsNorth;
    private final int[] cellPollen = new int[2];   // [south, north]
    private final int[] cellNectar = new int[2];

    private int held;
    private int launched, scored, missed, collected, tips;
    private double lastTipS = Double.NEGATIVE_INFINITY;
    private final List<Double> tipTimes = new ArrayList<>();
    private final List<String> contacts = new ArrayList<>();
    private double launchTimer, flowerTimer;
    private double driveIn, strafeIn;
    private double time;

    // ---- drawing only (AdvantageScope) ----
    static final double FLIGHT_S = 0.45;          // how long a drawn launch takes to arrive
    static final double LAUNCH_HEIGHT_IN = 16.0;  // PLACEHOLDER: where the pollen leaves the robot
    static final double CELL_AIM_Z_IN = 59.5;     // SPEC: middle of the up-cell opening (field file)
    private final List<double[]> flights = new ArrayList<>(); // {t0, x0, y0, x1, y1, z1, cell(-1 = loose)}
    private final List<double[]> spilled = new ArrayList<>(); // {x, y}

    // ---- physics mode ----
    static final double POLLEN_RADIUS_IN = 1.4;   // SPEC (field file)
    static final double MISS_AFTER_S = 2.5;       // a shot not seated this long after launch has missed
    private GamePieceTracker world;               // null: the rule-based launch above
    private final List<double[]> shots = new ArrayList<>(); // {id, launch time, state: 0 flying, 1 scored, 2 missed}
    private int stop;                             // -1 south cell up, +1 north up
    private double swingStart = Double.NaN;
    private double prevX = Double.NaN, prevY;
    private double pollenSpinRad, nectarSpinRad;

    public BiobuzzMatch(BioHorsSim sim, Json field, Json robot, boolean red, int preload) {
        this.sim = sim;
        this.red = red;
        this.held = preload;
        Json fp = robot.get("footprint").get("expandedIn"); // the longest the robot gets (flower bar out)
        robotLength = fp.num("lengthIn");
        robotWidth = fp.num("widthIn");
        robotHeight = robot.get("heightIn").num("value");
        Json cor = robot.get("footprint").get("centreOfRotationIn");
        corForward = cor.num("xIn");
        corLeft = cor.num("yIn");
        Json mouth = robot.get("mouths").at(0);
        mouthOffsetX = mouth.get("offsetIn").num("xIn");
        mouthOffsetY = mouth.get("offsetIn").num("yIn");
        mouthHalfWidth = mouth.num("widthIn") / 2.0;
        mouthHalfDepth = mouth.num("depthIn") / 2.0;

        Json table = field.get("rules").get("tipTable");
        tipTable = new int[table.size()];
        for (int i = 0; i < tipTable.length; i++) tipTable[i] = table.at(i).integer();

        String own = red ? "Red" : "Blue";
        double pivotX = 0;
        for (Json e : field.get("elements").items()) {
            String id = e.str("id");
            String kind = e.optText("kind", "");
            if (kind.equals("pollen") && id.startsWith("garden" + own)) {
                loosePollen.add(new double[] {e.num("xIn"), e.num("yIn"), e.num("radiusIn")});
            } else if (kind.equals("container") && "flower".equals(e.optText("container", ""))) {
                flowers.add(new Flower(id, e.num("xIn"), e.num("yIn"), e.get("holds").size()));
            } else if (id.equals("hive" + own)) {
                pivotX = e.get("pivotIn").num("xIn");
                upIsNorth = "NORTH".equals(e.str("upSide"));
                int nectar = e.get("holds").size();
                cellNectar[upIsNorth ? 1 : 0] = nectar; // the staged nectar sits in the up cell
            }
        }
        hiveX = pivotX;

        for (Json o : field.get("obstacles").items()) {
            String id = o.str("id");
            if ("box".equals(o.str("kind")) && o.num("minZIn") < robotHeight) {
                boxes.add(new double[] {o.num("minXIn"), o.num("maxXIn"), o.num("minYIn"), o.num("maxYIn")});
                boxNames.add(id);
            } else if ("strut".equals(o.str("kind"))) {
                Json a = o.get("fromIn");
                Json b = o.get("toIn");
                struts.add(new double[] {a.num("xIn"), a.num("yIn"), a.num("zIn"), b.num("xIn"), b.num("yIn"),
                        b.num("zIn"), o.num("radiusIn")});
                strutNames.add(id);
            }
        }
        for (Flower f : flowers) {
            for (int i = 0; i < boxNames.size(); i++) {
                if (boxNames.get(i).equals(f.id + "Foot")) f.setFoot(boxes.get(i));
            }
        }
    }

    /**
     * Launches become real balls in SimLoop's physics (see the class comment). Call before the
     * first step.
     */
    public BiobuzzMatch usePhysics() {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder().pieceMass("pollen", 1.0).build();
        world = new GamePieceTracker(4, FieldWorld.builder().perimeter(72, 72).physics(physics)
                .kinematicBody(BiobuzzHive.body(red)).build());
        world.setBodyMotion(BiobuzzHive.bodyName(red), BiobuzzHive.angleWithUp(upIsNorth), 0.0);
        stop = upIsNorth ? 1 : -1;
        return this;
    }

    /** One tick of the world, after the robot code and the plants. */
    public void step(double dt) {
        time += dt;
        double x = sim.drivePlant.getX();
        double y = sim.drivePlant.getY();
        double h = sim.drivePlant.getHeading();
        double fwdX = Math.cos(h), fwdY = Math.sin(h);
        double vx = Double.isNaN(prevX) ? 0 : (x - prevX) / dt, vy = Double.isNaN(prevX) ? 0 : (y - prevY) / dt;
        prevX = x;
        prevY = y;
        boolean intakeIn = sim.intakeLeft.getPower() > 0.1 && sim.intakeRight.getPower() > 0.1;

        // Driving vs strafing, from the plant's robot-frame velocities.
        driveIn += Math.hypot(sim.drivePlant.getForwardVelocity(), sim.drivePlant.getLateralVelocity()) * dt;
        strafeIn += Math.abs(sim.drivePlant.getLateralVelocity()) * dt;

        // ---- intake: garden pollen under the mouth ----
        if (intakeIn) {
            for (int i = loosePollen.size() - 1; i >= 0 && held < 4; i--) {
                if (mouthTouches(loosePollen.get(i), x, y, h)) {
                    loosePollen.remove(i);
                    held++;
                    collected++;
                }
            }
        }

        // ---- intake: flower retrieval opening ----
        Flower at = null;
        // Both servos really at DOWN. The right one is mounted mirrored (REVERSE), so its output is 1 - p.
        double down = org.firstinspires.ftc.teamcode.subsystems.MechanismConstants.FLOWER_BAR_DOWN;
        boolean barDown = Math.abs(sim.flowerLeftPlant.getPosition() - down) < 0.05
                && Math.abs(sim.flowerRightPlant.getPosition() - (1.0 - down)) < 0.05;
        if (intakeIn && held < 4 && barDown) {
            double mx = x + fwdX * (mouthOffsetX + mouthHalfDepth) - fwdY * mouthOffsetY;
            double my = y + fwdY * (mouthOffsetX + mouthHalfDepth) + fwdX * mouthOffsetY;
            for (Flower f : flowers) {
                if (f.count > 0 && f.reachable(mx, my, fwdX, fwdY)) at = f;
            }
        }
        if (at != null) {
            flowerTimer += dt;
            while (flowerTimer >= FLOWER_S_PER_PIECE && at.count > 0 && held < 4) {
                flowerTimer -= FLOWER_S_PER_PIECE;
                at.count--;
                held++;
                collected++;
            }
        } else {
            flowerTimer = 0;
        }

        // ---- launches ----
        boolean gateOpen = sim.pollenGatePlant.getPosition() >= GATE_OPEN_THRESHOLD;
        boolean feeding = gateOpen && intakeIn && sim.pollenShooterRpm() >= LAUNCH_MIN_RPM && held > 0;
        if (feeding) {
            launchTimer += dt;
            while (launchTimer >= LAUNCH_S_PER_PIECE && held > 0) {
                launchTimer -= LAUNCH_S_PER_PIECE;
                held--;
                launched++;
                if (world != null) {
                    double[] l = ShotModel.pollenLaunch(x, y, h, sim.pollenShooterRpm(), vx, vy);
                    int id = world.addHeldPiece("pollen", POLLEN_RADIUS_IN);
                    world.launch(id, "shot", l[0], l[1], l[2], l[3], l[4], l[5]);
                    shots.add(new double[] {id, time, 0});
                } else if (time - lastTipS <= TIP_CATCH_S && legalInto(x, y, h, !upIsNorth)) {
                    // The cell that just tipped is only starting to swing: a shot right behind the
                    // tipping one still goes in, and spills with the rest of its load.
                    int cell = upIsNorth ? 0 : 1;
                    scored++;
                    spill(cell, 1);
                } else if (launchIsLegal(x, y, h)) {
                    int up = upIsNorth ? 1 : 0;
                    double cellY = (upIsNorth ? 1 : -1) * FieldFace.OUTER_FACE_IN / 2.0;
                    flights.add(new double[] {time, x, y, hiveX, cellY, CELL_AIM_Z_IN, up});
                    cellPollen[up]++;
                    scored++;
                    int need = tipTable[Math.min(cellNectar[up], tipTable.length - 1)];
                    if (cellPollen[up] >= need) {
                        tips++;
                        tipTimes.add(time);
                        lastTipS = time;
                        spill(up, cellPollen[up]);
                        cellPollen[up] = 0;
                        cellNectar[up] = 0;
                        upIsNorth = !upIsNorth;
                    }
                } else {
                    missed++;
                    flights.add(new double[] {time, x, y, x + 40 * Math.cos(h), y + 40 * Math.sin(h), 1.4, -1});
                    missLog.add(String.format(java.util.Locale.ROOT, "t=%.2f x=%.1f y=%.1f h=%.1fdeg (%s)",
                            time, x, y, Math.toDegrees(h), missReason(x, y, h)));
                }
            }
        } else {
            launchTimer = 0;
        }

        if (world != null) stepPhysics(dt);

        pollenSpinRad = (pollenSpinRad + sim.pollenShooterRpm() * 2 * Math.PI / 60.0 * dt) % (2 * Math.PI);
        nectarSpinRad = (nectarSpinRad + sim.nectarShooterRpm() * 2 * Math.PI / 60.0 * dt) % (2 * Math.PI);

        checkContacts(x, y, h, intakeIn);

        // The pollen really aboard press on the intake rollers: running inward, both motors slow
        // (the robot's IntakeLoadSensor reads this through their encoders). Applied next tick.
        double load = intakeIn ? -INTAKE_LOAD_PER_PIECE * held : 0.0;
        sim.intakeLeft.setExternalTorqueFraction(load);
        sim.intakeRight.setExternalTorqueFraction(load);

        logField(x, y, h);
        Logger.recordOutput("Match/held", held);
        Logger.recordOutput("Match/launched", launched);
        Logger.recordOutput("Match/scored", scored);
        Logger.recordOutput("Match/missed", missed);
        Logger.recordOutput("Match/tips", tips);
        Logger.recordOutput("Match/upCellNorth", upIsNorth);
        // AdvantageScope's hives are fixed field art and never tip, so mark the cell that is really
        // up: a pose at its opening (draw it as Axes or a Cone), which moves when the hive tips.
        Logger.recordOutput("Field/UpCell", Pose3d.struct, AdvantageScopeFrame.piece(hiveX,
                (upIsNorth ? 1 : -1) * FieldFace.OUTER_FACE_IN / 2.0, CELL_AIM_Z_IN));
        Logger.recordOutput("Truth/xIn", x);
        Logger.recordOutput("Truth/yIn", y);
        Logger.recordOutput("Truth/headingRad", h);
        Logger.recordOutput("Shooter/pollenRpm", sim.pollenShooterRpm());
        Logger.recordOutput("Gate/pollen", sim.pollenGatePlant.getPosition());
        Logger.recordOutput("FlowerBar/position", sim.flowerLeftPlant.getPosition());
        Logger.recordOutput("Intake/power", sim.intakeLeft.getPower());
    }

    private void stepPhysics(double dt) {
        world.advance(dt);
        String hive = BiobuzzHive.bodyName(red);
        for (double[] s : shots) {
            if (s[2] != 0) continue;
            GamePiece p = world.getPiece((int) s[0]);
            String cell = p.getSeatedCellName();
            if (cell != null && cell.startsWith(hive)) {
                s[2] = 1;
                scored++;
            } else if (time - s[1] > MISS_AFTER_S) {
                s[2] = 2;
                missed++;
                missLog.add(String.format(java.util.Locale.ROOT, "t=%.2f launched, not seated; at (%.1f, %.1f, %.1f)",
                        s[1], p.getFieldXInches(), p.getFieldYInches(), p.getFieldZInches()));
            }
        }
        // A tip: the hive leaves its stop and reaches the other one.
        double a = world.getBodyAngleRad(hive), tilt = BiobuzzHive.TILT_RAD, eps = Math.toRadians(1.0);
        if (Math.abs(a - stop * tilt) > Math.toRadians(2.0)) {
            if (Double.isNaN(swingStart)) swingStart = time;
        } else {
            swingStart = Double.NaN;
        }
        if (Math.abs(a + stop * tilt) < eps) {
            tips++;
            tipTimes.add(Double.isNaN(swingStart) ? time : swingStart);
            stop = -stop;
            upIsNorth = stop > 0;
            swingStart = Double.NaN;
        }
        Logger.recordOutput("Match/hiveAngleDeg", Math.toDegrees(a));
        Logger.recordOutput("Field/Hive", Pose3d.struct, AdvantageScopeFrame.hive(hiveX, BiobuzzHive.AXLE_Z_IN, a));
    }

    /** Every ball launched in physics mode, in flight, seated or on the floor (empty otherwise). */
    public List<GamePiece> shotPieces() {
        List<GamePiece> out = new ArrayList<>();
        if (world != null) for (double[] s : shots) out.add(world.getPiece((int) s[0]));
        return out;
    }

    public boolean physics() {
        return world != null;
    }

    private final List<String> missLog = new ArrayList<>();

    public List<String> missLog() { return missLog; }

    private String missReason(double x, double y, double h) {
        boolean ownHalf = red ? x < 0 : x > 0;
        double face = FieldFace.OUTER_FACE_IN;
        boolean outboard = upIsNorth ? y > face : y < -face;
        double cellY = (upIsNorth ? 1 : -1) * face / 2.0;
        double bearing = Math.atan2(cellY - y, hiveX - x);
        double off = Math.toDegrees(Math.abs(Math.atan2(Math.sin(bearing - h), Math.cos(bearing - h))));
        return (ownHalf ? "" : "not own half; ") + (outboard ? "" : "not outboard of the up cell; ")
                + String.format(java.util.Locale.ROOT, "%.1f deg off the cell", off);
    }

    /** A tipped cell's load, drawn on the floor outboard of it on its own side. */
    private void spill(int cell, int count) {
        // The pollen still flying into this cell is part of the load that tips: it is drawn in the
        // spill from now on, not in the air as well.
        flights.removeIf(f -> f[6] == cell);
        double side = cell == 1 ? 1 : -1;
        for (int i = 0; i < count; i++) {
            double a = (i - (count - 1) / 2.0) * 0.18;
            double r = 26 + 5 * (i % 3);
            spilled.add(new double[] {hiveX + r * Math.sin(a), side * (r * Math.cos(a))});
        }
    }

    private void logField(double x, double y, double h) {
        Logger.recordOutput("Field/Robot", Pose2d.struct, AdvantageScopeFrame.pose2d(x, y, h));
        // Order matches the model's components: model_0 pollen, model_1 nectar.
        Logger.recordOutput("Field/RobotComponents", new Pose3d[] {
                AdvantageScopeFrame.robotFlywheel(POLLEN_AXLE_FORWARD_IN, POLLEN_AXLE_UP_IN,
                        FLYWHEEL_SPIN_SIGN * pollenSpinRad),
                AdvantageScopeFrame.robotFlywheel(NECTAR_AXLE_FORWARD_IN, NECTAR_AXLE_UP_IN,
                        FLYWHEEL_SPIN_SIGN * nectarSpinRad)});
        List<Pose3d> pieces = new ArrayList<>();
        for (double[] p : loosePollen) pieces.add(AdvantageScopeFrame.piece(p[0], p[1], p[2]));
        for (Flower f : flowers) {
            for (int i = 0; i < f.count; i++) pieces.add(AdvantageScopeFrame.piece(f.x, f.y, 1.6 + 2.9 * i));
        }
        for (int i = 0; i < held; i++) pieces.add(AdvantageScopeFrame.piece(x, y, 8.0 + 3.0 * i));
        if (world != null) {
            for (GamePiece p : shotPieces()) {
                pieces.add(AdvantageScopeFrame.piece(p.getFieldXInches(), p.getFieldYInches(), p.getFieldZInches()));
            }
            Logger.recordOutput("Field/Pollen", pieces.toArray(new Pose3d[0]));
            return;
        }
        int[] arriving = new int[2];
        for (int i = flights.size() - 1; i >= 0; i--) {
            double[] f = flights.get(i);
            double u = (time - f[0]) / FLIGHT_S;
            if (u >= 1) {
                if (f[6] < 0) spilled.add(new double[] {f[3], f[4]});
                flights.remove(i);
                continue;
            }
            if (f[6] >= 0) arriving[(int) f[6]]++;
            double px = f[1] + u * (f[3] - f[1]);
            double py = f[2] + u * (f[4] - f[2]);
            double pz = LAUNCH_HEIGHT_IN + u * (f[5] - LAUNCH_HEIGHT_IN) + 4 * 14.0 * u * (1 - u);
            pieces.add(AdvantageScopeFrame.piece(px, py, pz));
        }
        for (int c = 0; c < 2; c++) {
            double cy = (c == 1 ? 1 : -1) * FieldFace.OUTER_FACE_IN / 2.0;
            int shown = Math.max(0, cellPollen[c] - arriving[c]);
            for (int i = 0; i < shown; i++) {
                pieces.add(AdvantageScopeFrame.piece(hiveX + ((i % 2) * 3.0 - 1.5), cy + ((i / 2 % 2) * 3.0 - 1.5),
                        CELL_AIM_Z_IN - 4 + 3.0 * (i / 4)));
            }
        }
        for (double[] p : spilled) pieces.add(AdvantageScopeFrame.piece(p[0], p[1], 1.4));
        Logger.recordOutput("Field/Pollen", pieces.toArray(new Pose3d[0]));
    }

    private boolean launchIsLegal(double x, double y, double h) {
        return legalInto(x, y, h, upIsNorth);
    }

    /** A launch from (x, y, h) would go into the north (or south) cell, were it up. */
    private boolean legalInto(double x, double y, double h, boolean north) {
        boolean ownHalf = red ? x < 0 : x > 0;
        double face = FieldFace.OUTER_FACE_IN;
        boolean outboard = north ? y > face : y < -face;
        double cellY = (north ? 1 : -1) * face / 2.0;
        double bearing = Math.atan2(cellY - y, hiveX - x);
        double off = Math.abs(Math.atan2(Math.sin(bearing - h), Math.cos(bearing - h)));
        return ownHalf && outboard && Math.toDegrees(off) <= LAUNCH_CONE_DEG;
    }

    private boolean mouthTouches(double[] p, double x, double y, double h) {
        double dx = p[0] - x, dy = p[1] - y;
        double fwd = dx * Math.cos(h) + dy * Math.sin(h);
        double left = -dx * Math.sin(h) + dy * Math.cos(h);
        return Math.abs(fwd - mouthOffsetX) <= mouthHalfDepth + p[2]
                && Math.abs(left - mouthOffsetY) <= mouthHalfWidth + p[2];
    }

    // ------------------------------------------------------------------ contact

    private void checkContacts(double x, double y, double h, boolean intakeIn) {
        double[][] c = corners(x, y, h);
        for (double[] p : c) {
            if (Math.abs(p[0]) > 72.0 + CONTACT_TOLERANCE_IN || Math.abs(p[1]) > 72.0 + CONTACT_TOLERANCE_IN) {
                note("wall");
                break;
            }
        }
        for (int i = 0; i < boxes.size(); i++) {
            double[] b = boxes.get(i);
            if (obbHitsBox(x, y, h, b)) {
                String name = boxNames.get(i);
                boolean intended = intakeIn && name.startsWith("flower") && name.endsWith("Foot");
                note(intended ? name + " (intended: collecting)" : name);
            }
        }
        for (int i = 0; i < struts.size(); i++) {
            double[] s = struts.get(i);
            for (int k = 0; k <= 8; k++) {
                double t = k / 8.0;
                double z = s[2] + t * (s[5] - s[2]);
                if (z > robotHeight) continue;
                double px = s[0] + t * (s[3] - s[0]);
                double py = s[1] + t * (s[4] - s[1]);
                if (pointInFootprint(px, py, x, y, h, s[6] - CONTACT_TOLERANCE_IN)) {
                    note(strutNames.get(i));
                    break;
                }
            }
        }
    }

    private void note(String what) {
        String entry = String.format(java.util.Locale.ROOT, "%s @%.2fs (x=%.1f y=%.1f h=%.0fdeg)", what, time,
                sim.drivePlant.getX(), sim.drivePlant.getY(), Math.toDegrees(sim.drivePlant.getHeading()));
        // One entry per obstacle per contiguous contact.
        if (contacts.isEmpty() || !contacts.get(contacts.size() - 1).startsWith(what + " @")
                || time - lastContactTime > 0.1) {
            contacts.add(entry);
        }
        lastContactTime = time;
    }

    private double lastContactTime = -1;

    private double[][] corners(double x, double y, double h) {
        double hl = robotLength / 2, hw = robotWidth / 2;
        double c = Math.cos(h), s = Math.sin(h);
        // The box centre is the pose minus the turning centre's offset inside the box (as Zenith does).
        x -= corForward * c - corLeft * s;
        y -= corForward * s + corLeft * c;
        double[][] out = new double[4][];
        int i = 0;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                double fx = sx * hl, fy = sy * hw;
                out[i++] = new double[] {x + fx * c - fy * s, y + fx * s + fy * c};
            }
        }
        return out;
    }

    private boolean pointInFootprint(double px, double py, double x, double y, double h, double grow) {
        x -= corForward * Math.cos(h) - corLeft * Math.sin(h);
        y -= corForward * Math.sin(h) + corLeft * Math.cos(h);
        double dx = px - x, dy = py - y;
        double fwd = dx * Math.cos(h) + dy * Math.sin(h);
        double left = -dx * Math.sin(h) + dy * Math.cos(h);
        return Math.abs(fwd) <= robotLength / 2 + grow && Math.abs(left) <= robotWidth / 2 + grow;
    }

    /** Separating-axis test between the robot's footprint and an axis-aligned box, with tolerance. */
    private boolean obbHitsBox(double x, double y, double h, double[] b) {
        double tol = CONTACT_TOLERANCE_IN;
        double[][] rc = corners(x, y, h);
        double[][] bc = {{b[0], b[2]}, {b[1], b[2]}, {b[0], b[3]}, {b[1], b[3]}};
        double[][] axes = {{1, 0}, {0, 1}, {Math.cos(h), Math.sin(h)}, {-Math.sin(h), Math.cos(h)}};
        for (double[] a : axes) {
            double rMin = Double.MAX_VALUE, rMax = -Double.MAX_VALUE, bMin = Double.MAX_VALUE, bMax = -Double.MAX_VALUE;
            for (double[] p : rc) {
                double d = p[0] * a[0] + p[1] * a[1];
                rMin = Math.min(rMin, d);
                rMax = Math.max(rMax, d);
            }
            for (double[] p : bc) {
                double d = p[0] * a[0] + p[1] * a[1];
                bMin = Math.min(bMin, d);
                bMax = Math.max(bMax, d);
            }
            if (rMax < bMin + tol || bMax < rMin + tol) return false;
        }
        return true;
    }

    // ------------------------------------------------------------------ results

    public int held() { return held; }
    public int launched() { return launched; }
    public int scored() { return scored; }
    public int missed() { return missed; }
    public int collected() { return collected; }
    public int tips() { return tips; }
    public List<Double> tipTimes() { return tipTimes; }
    public List<String> contacts() { return contacts; }
    public double strafeFraction() { return driveIn <= 1e-9 ? 0 : strafeIn / driveIn; }
    public double drivenIn() { return driveIn; }

    /** Contacts other than the intended kind (pressing the intake to a flower to collect). */
    public List<String> unintendedContacts() {
        List<String> out = new ArrayList<>();
        for (String c : contacts) if (!c.contains("(intended")) out.add(c);
        return out;
    }

    /** The cell geometry Zenith's BIOBUZZ ledger uses (SPEC: FIRST field CAD). */
    static final class FieldFace {
        static final double OUTER_FACE_IN = 18.528;
    }

    /** A flower: its pollen column and the face of its foot the intake pulls from. */
    private final class Flower {
        final String id;
        final double x, y;
        int count;
        double openX, openY, inX, inY; // opening point and its inward-to-the-field normal

        Flower(String id, double x, double y, int count) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.count = count;
        }

        void setFoot(double[] b) {
            // The retrieval opening is on the field side: the foot face farthest from the wall.
            if (Math.abs(x) > Math.abs(y)) {
                inX = -Math.signum(x);
                inY = 0;
                openX = x < 0 ? b[1] : b[0];
                openY = y;
            } else {
                inX = 0;
                inY = -Math.signum(y);
                openX = x;
                openY = y < 0 ? b[3] : b[2];
            }
        }

        boolean reachable(double mx, double my, double fwdX, double fwdY) {
            boolean facing = -(fwdX * inX + fwdY * inY) >= Math.cos(Math.toRadians(FACING_CONE_DEG));
            return facing && Math.hypot(mx - openX, my - openY) <= OPENING_REACH_IN;
        }
    }
}
