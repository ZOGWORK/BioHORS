package org.firstinspires.ftc.teamcode.zenith;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.firstinspires.ftc.teamcode.sim.BioHorsSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The Zenith score command, at a scoring spot, with a flywheel that never spins (nothing steps the
 * sim): at the spin-up timeout it must skip the volley with the gate shut and the pollen kept, not
 * feed pollen into a stopped wheel.
 */
class ScoreCommandSpinUpTest {

    private double spinup;

    @BeforeEach
    void shortenSpinUp() {
        spinup = ScoreCommand.SPINUP_TIMEOUT_S;
        ScoreCommand.SPINUP_TIMEOUT_S = 0.05;
    }

    @AfterEach
    void restore() {
        ScoreCommand.SPINUP_TIMEOUT_S = spinup;
    }

    @Test
    void aFlywheelThatNeverSpinsUpSkipsTheVolley() throws InterruptedException {
        BioHorsSim sim = new BioHorsSim();
        BioHorsRobot robot = new BioHorsRobot(sim.hardwareMap, null);
        PollenEstimate pollen = new PollenEstimate(4);
        ZenithSubsystems.Intake intake = new ZenithSubsystems.Intake(robot, pollen);
        ZenithSubsystems.Launcher launcher = new ZenithSubsystems.Launcher(robot);
        ScoreCommand score = new ScoreCommand(launcher, intake, pollen, 4, () -> true);

        score.initialize();
        long deadline = System.nanoTime() + 3_000_000_000L;
        while (!score.isFinished() && System.nanoTime() < deadline) {
            score.execute();
            intake.periodic();
            assertTrue(robot.pollenGate.isClosed(), "the gate opened onto a stopped flywheel");
            assertTrue(sim.intakeLeft.getPower() <= 0, "nothing is fed");
            Thread.sleep(5);
        }
        score.end(false);

        assertTrue(score.isFinished(), "the volley gives up at the timeout");
        assertEquals(1, launcher.spinUpFailures());
        assertEquals(4, pollen.held(), "the pollen stay aboard");
    }
}
