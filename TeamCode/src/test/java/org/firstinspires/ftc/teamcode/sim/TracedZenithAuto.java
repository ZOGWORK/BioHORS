package org.firstinspires.ftc.teamcode.sim;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.zenith.BioHorsZenithRobot;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.horizon36596.zenith.AutoSource;
import org.horizon36596.zenith.solverslib.AutoFromFile;
import org.horizon36596.zenith.solverslib.AutoTraceWriter;
import org.horizon36596.zenith.solverslib.ZenithRobot;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;

/**
 * The OpMode `zenith deploy` generates for an auto (an AutoFromFile that builds BioHorsZenithRobot),
 * with Zenith's trace recorder around it. The only differences are what a sim needs: the auto's
 * name and folder come from the test, every step is wrapped in AutoTraceWriter.traced, and every
 * loop records the follower's pose (belief) and the plant's pose (truth).
 *
 * SimLoop builds this class inside its sim-clock class loader (OpModeRunner.forTeamClass), so the
 * test hands it what it needs through OpMode.blackboard, using only SDK, JDK and SimLoop types.
 * Pattern from Horizon's example robot (Horizon-Library-Examples, TracedAuto).
 */
public final class TracedZenithAuto extends AutoFromFile {

    static final String AUTO = "zenith.sim.auto";
    static final String SOURCE_DIR = "zenith.sim.sourceDir";
    static final String TRACE_FILE = "zenith.sim.traceFile";
    static final String TICK_S = "zenith.sim.tickS";
    static final String DRIVE_PLANT = "zenith.sim.drivePlant";

    private AutoTraceWriter trace;
    private MecanumDrivePlant plant;
    private long startNanos;
    private boolean started;

    @Override
    protected String autoName() {
        return (String) blackboard.get(AUTO);
    }

    @Override
    protected ZenithRobot createRobot() {
        return new BioHorsZenithRobot() {
            @Override
            public Command step(String id, Command body) {
                return trace.traced(id, body);
            }
        };
    }

    @Override
    public void initialize() {
        trace = new AutoTraceWriter(autoName(), (Double) blackboard.get(TICK_S));
        plant = (MecanumDrivePlant) blackboard.get(DRIVE_PLANT);
        setAutoSource(AutoSource.directory(new File((String) blackboard.get(SOURCE_DIR))));
        super.initialize();
    }

    /** Trace time 0 is the whistle: the moment the routine is scheduled. */
    @Override
    public void startRoutine() {
        startNanos = System.nanoTime();
        started = true;
        trace.tick(0);
        super.startRoutine();
        record();
    }

    @Override
    public void run() {
        trace.tick((System.nanoTime() - startNanos) / 1e9);
        super.run();
        record();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            try {
                trace.write(new File((String) blackboard.get(TRACE_FILE)));
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }
    }

    private void record() {
        if (!started) return;
        Pose2d belief = currentPose();
        trace.pose(belief.getX(), belief.getY(), belief.getHeading());
        trace.truthPose(plant.getX(), plant.getY(), plant.getHeading());
    }
}
