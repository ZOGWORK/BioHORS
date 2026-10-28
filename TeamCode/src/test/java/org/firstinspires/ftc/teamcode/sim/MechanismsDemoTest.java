package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * The robot model's moving parts, for AdvantageScope's 3D field: the robot stands still in open
 * floor while the real IntakeController and FlowerBarController run a fixed sequence, and the log
 * carries the intake rollers' and the flower bar's poses (Field/RobotComponents, model_2..6).
 *
 * RLOG: TeamCode/build/sim/mechanisms-demo.rlog. Checked from the log: the bar sits at its CAD pose
 * (down) while DOWN, folds back past upright when STOWED, a little up when SHOOTING; the rollers
 * turn while the intake runs and stop when it stops.
 */
class MechanismsDemoTest {

    static final double STEP_S = 1.5;
    static final String[] STEPS = {"STOWED", "DOWN_INTAKE", "SHOOTING_INTAKE", "STOWED_OFF"};

    /** One step of STEPS every STEP_S, by loop count (0.02 s a loop). */
    public static class Sequence extends OpMode {
        private BioHorsRobot robot;
        private int loops;

        @Override
        public void init() {
            robot = new BioHorsRobot(hardwareMap, telemetry);
        }

        @Override
        public void loop() {
            int step = Math.min(STEPS.length - 1, (int) (loops++ * OpModeRunner.DEFAULT_TICK_SECONDS / STEP_S));
            String name = STEPS[step];
            robot.flowerBar.setDown(name.startsWith("DOWN"));
            robot.flowerBar.setShooting(name.startsWith("SHOOTING"));
            if (name.endsWith("INTAKE")) robot.intake.intake(); else robot.intake.stop();
            robot.updateMechanisms();
            Logger.recordOutput("Demo/step", name);
        }

        @Override
        public void stop() {
            robot.stop();
        }
    }

    @Test
    void theIntakeSpinsAndTheFlowerBarMoves() throws IOException {
        File root = projectRoot();
        File assets = new File(root, "TeamCode/src/main/assets/autos");
        BioHorsSim sim = new BioHorsSim();
        BiobuzzMatch match = new BiobuzzMatch(sim, readJson(new File(assets, "biobuzz.field.json")),
                readJson(new File(assets, "robot.json")), true, 0);
        sim.placeAt(0, -44, 0);   // open floor south of the hive frame, facing +X
        Path rlog = new File(root, "TeamCode/build/sim/mechanisms-demo.rlog").toPath();
        Files.createDirectories(rlog.getParent());

        OpModeRunner.forTeamClass(Sequence.class, sim.hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.")
                .initLoopTicks(5)
                .activeTicks((int) Math.round(STEPS.length * STEP_S / OpModeRunner.DEFAULT_TICK_SECONDS))
                .eachTick(dt -> {
                    sim.step(dt);
                    match.step(dt);
                })
                .runLogged(BioHorsSim.ROBOT_CONFIG, rlog);

        // Per step, from the log: the bar angle at the end, and how far the rollers turned in its
        // second half (after the motors have had time to speed up or stop).
        List<Map<String, String>> frames = RlogDecodedCompare.decode(rlog);
        List<String> steps = new ArrayList<>();
        List<Double> bar = new ArrayList<>(), spin = new ArrayList<>();
        for (Map<String, String> f : frames) {
            String step = value(f, "RealOutputs/Demo/step"), b = value(f, "RealOutputs/Viz/flowerBarDeg"),
                    s = value(f, "RealOutputs/Viz/intakeSpinDeg");
            if (step == null || b == null || s == null) continue;
            steps.add(step);
            bar.add(Double.parseDouble(b));
            spin.add(Double.parseDouble(s));
        }
        for (String name : STEPS) {
            int first = steps.indexOf(name), last = steps.lastIndexOf(name);
            assertTrue(first >= 0, "no frames for " + name);
            double endBar = bar.get(last);
            double turned = 0;
            for (int i = (first + last) / 2 + 1; i <= last; i++) {
                double d = spin.get(i) - spin.get(i - 1);
                turned += Math.abs(Math.IEEEremainder(d, 360.0));
            }
            System.out.printf(java.util.Locale.ROOT, "MECHANISMS %-16s bar %7.1f deg, rollers turned %7.1f deg%n",
                    name, endBar, turned);
            boolean intakeOn = name.endsWith("INTAKE");
            assertTrue(intakeOn ? turned > 360 : turned < 1, name + ": rollers turned " + turned + " deg");
            if (name.startsWith("DOWN")) assertEquals(0.0, endBar, 1.0, "DOWN is the CAD pose");
            if (name.startsWith("STOWED")) {
                assertEquals(Math.toDegrees(BiobuzzMatch.flowerBarAngleRad(
                        org.firstinspires.ftc.teamcode.subsystems.MechanismConstants.FLOWER_BAR_STOWED)), endBar, 1.0, name + " bar");
                assertTrue(endBar < -95, "STOWED folds the bar back past upright: " + endBar);
            }
            if (name.startsWith("SHOOTING")) assertTrue(endBar < -5 && endBar > -45, "SHOOTING lifts it a little: " + endBar);
        }
        java.util.Set<String> keys = RlogDecodedCompare.decodedKeys(rlog);
        assertTrue(keys.stream().anyMatch(x -> x.endsWith("Field/RobotComponents")), "RLOG has no Field/RobotComponents");
    }

    private static String value(Map<String, String> frame, String key) {
        for (Map.Entry<String, String> e : frame.entrySet()) {
            if (e.getKey().endsWith(key)) return e.getValue();
        }
        return null;
    }

    private static Json readJson(File file) throws IOException {
        try (InputStream in = new FileInputStream(file)) {
            return Json.parse(in, file.getName());
        }
    }

    private static File projectRoot() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) dir = dir.getParentFile();
        if (dir == null) throw new IllegalStateException("no zenith.json above " + System.getProperty("user.dir"));
        return dir;
    }
}
