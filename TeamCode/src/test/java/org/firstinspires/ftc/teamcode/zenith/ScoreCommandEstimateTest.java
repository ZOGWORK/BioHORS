package org.firstinspires.ftc.teamcode.zenith;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.firstinspires.ftc.teamcode.sim.BioHorsSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Feeding the shooter runs the intake inward, but it is not collecting: a volley of N starting with
 * N pollen must leave the estimate at 0 (holdingPiece false, hopperEmpty true).
 *
 * Runs ScoreCommand by hand on the real clock with the REAL feed timings (0.4 s per pollen plus
 * 0.2 s): with shorter ones, feeding-as-collecting adds under one pollen and floor() hides the bug.
 * Only the spin-up wait is shortened (the fake shooter is never commanded to reach speed here).
 */
class ScoreCommandEstimateTest {

    private double spinup;

    @BeforeEach
    void shortenSpinUp() {
        spinup = ScoreCommand.SPINUP_TIMEOUT_S;
        ScoreCommand.SPINUP_TIMEOUT_S = 0.01;
    }

    @AfterEach
    void restoreSpinUp() {
        ScoreCommand.SPINUP_TIMEOUT_S = spinup;
    }

    @ParameterizedTest
    @ValueSource(ints = {1, 2, 3, 4})
    void aVolleyOfNEmptiesARobotHoldingN(int n) throws InterruptedException {
        BioHorsSim sim = new BioHorsSim();
        BioHorsRobot robot = new BioHorsRobot(sim.hardwareMap, null);
        PollenEstimate pollen = new PollenEstimate(n);
        ZenithSubsystems.Intake intake = new ZenithSubsystems.Intake(robot, pollen);
        ZenithSubsystems.Launcher launcher = new ZenithSubsystems.Launcher(robot);
        ScoreCommand score = new ScoreCommand(launcher, intake, pollen, n, () -> true);

        score.initialize();
        long deadline = System.nanoTime() + 10_000_000_000L;
        boolean fed = false;
        while (!score.isFinished() && System.nanoTime() < deadline) {
            score.execute();
            intake.periodic();
            fed |= sim.intakeLeft.getPower() > 0;
            Thread.sleep(5);
        }
        score.end(false);

        assertTrue(fed, "the volley should run the intake to feed");
        assertTrue(score.isFinished(), "the volley should end");
        assertEquals(0, pollen.held(), "a volley of " + n + " from " + n + " leaves none");
        assertTrue(pollen.empty());
        assertFalse(pollen.full());
    }

    @org.junit.jupiter.api.Test
    void offAScoringSpotTheGateStaysShutAndTheVolleyIsSkipped() throws InterruptedException {
        double savedWait = org.firstinspires.ftc.teamcode.auto.ShootingPose.WAIT_FOR_POSE_S;
        org.firstinspires.ftc.teamcode.auto.ShootingPose.WAIT_FOR_POSE_S = 0.05;
        try {
            BioHorsSim sim = new BioHorsSim();
            BioHorsRobot robot = new BioHorsRobot(sim.hardwareMap, null);
            PollenEstimate pollen = new PollenEstimate(4);
            ZenithSubsystems.Intake intake = new ZenithSubsystems.Intake(robot, pollen);
            ZenithSubsystems.Launcher launcher = new ZenithSubsystems.Launcher(robot);
            ScoreCommand score = new ScoreCommand(launcher, intake, pollen, 4, () -> false);
            score.initialize();
            long deadline = System.nanoTime() + 3_000_000_000L;
            while (!score.isFinished() && System.nanoTime() < deadline) {
                score.execute();
                intake.periodic();
                assertTrue(robot.pollenGate.isClosed(), "the gate opened off a scoring spot");
                Thread.sleep(5);
            }
            score.end(false);
            assertTrue(score.isFinished(), "the volley should give up");
            assertEquals(4, pollen.held(), "the pollen stay aboard");
        } finally {
            org.firstinspires.ftc.teamcode.auto.ShootingPose.WAIT_FOR_POSE_S = savedWait;
        }
    }
}
