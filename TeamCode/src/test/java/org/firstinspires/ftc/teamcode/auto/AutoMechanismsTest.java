package org.firstinspires.ftc.teamcode.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.firstinspires.ftc.teamcode.sim.BioHorsSim;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.firstinspires.ftc.teamcode.zenith.ScoreCommand;
import org.horizon36596.simloop.opmode.RecordingTelemetry;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * A volley is skipped only when the intake sensor has read the robot empty (as MAINHORS26's auto
 * skipped a shot with no balls). Not knowing, or the sensor turned off, still shoots.
 */
class AutoMechanismsTest {

    private final boolean savedEnabled = MechanismConstants.INTAKE_SENSOR_ENABLED;
    private final double savedMinFraction = MechanismConstants.SPINUP_MIN_FRACTION;

    /**
     * Nothing here steps the sim, so the fake flywheel never spins: the spin-up check is switched
     * off (any speed will do at the timeout) except in the test of the check itself.
     */
    @BeforeEach
    void anySpeedWillDo() {
        MechanismConstants.SPINUP_MIN_FRACTION = 0.0;
    }

    @AfterEach
    void restore() {
        MechanismConstants.INTAKE_SENSOR_ENABLED = savedEnabled;
        MechanismConstants.SPINUP_MIN_FRACTION = savedMinFraction;
    }

    @Test
    void aFlywheelThatNeverSpinsUpSkipsTheVolleyAndKeepsThePollen() throws InterruptedException {
        MechanismConstants.SPINUP_MIN_FRACTION = savedMinFraction;
        double savedSpin = ScoreCommand.SPINUP_TIMEOUT_S;
        ScoreCommand.SPINUP_TIMEOUT_S = 0.05;
        try {
            BioHorsRobot robot = robot();
            AutoMechanisms mech = new AutoMechanisms(robot, 4, () -> true);   // at a scoring spot
            mech.pollen().sensed(4);
            mech.volley(4);
            long deadline = System.nanoTime() + 3_000_000_000L;
            while (!mech.volleyDone() && System.nanoTime() < deadline) {
                mech.update();
                assertTrue(robot.pollenGate.isClosed(), "the gate opened onto a stopped flywheel");
                Thread.sleep(5);
            }
            assertTrue(mech.volleyDone(), "the volley should give up at the timeout");
            assertEquals(1, mech.volleysSkipped());
            assertEquals(1, mech.spinUpFailures());
            assertEquals(4, mech.pollen().held(), "the pollen stay aboard for a later volley");
        } finally {
            ScoreCommand.SPINUP_TIMEOUT_S = savedSpin;
        }
    }

    static BioHorsRobot robot() {
        return new BioHorsRobot(new BioHorsSim().hardwareMap, new RecordingTelemetry());
    }

    @Test
    void aVolleyIsSkippedWhenTheSensorReadsEmpty() {
        BioHorsRobot robot = robot();
        AutoMechanisms mech = new AutoMechanisms(robot, 0, () -> true);
        robot.intakeLoad.emptied();          // the sensor has read: empty
        mech.volley(4);
        assertTrue(mech.volleyDone(), "the auto can move straight on");
        assertEquals(1, mech.volleysSkipped());
        assertEquals(AutoMechanisms.PollenState.IDLE, mech.pollenState());
        assertTrue(robot.pollenGate.isClosed(), "nothing is fed");
    }

    @Test
    void anUnknownReadingStillShoots() {
        BioHorsRobot robot = robot();
        AutoMechanisms mech = new AutoMechanisms(robot, 0, () -> true);
        assertFalse(robot.intakeLoad.hasReading());
        mech.volley(4);
        assertEquals(AutoMechanisms.PollenState.SPIN_UP, mech.pollenState());
        assertEquals(0, mech.volleysSkipped());
    }

    @Test
    void withTheSensorOffItStillShoots() {
        MechanismConstants.INTAKE_SENSOR_ENABLED = false;
        BioHorsRobot robot = robot();
        AutoMechanisms mech = new AutoMechanisms(robot, 0, () -> true);
        robot.intakeLoad.emptied();
        mech.volley(4);
        assertEquals(AutoMechanisms.PollenState.SPIN_UP, mech.pollenState());
        assertEquals(0, mech.volleysSkipped());
    }

    @Test
    void aLoadedRobotShoots() {
        BioHorsRobot robot = robot();
        AutoMechanisms mech = new AutoMechanisms(robot, 4, () -> true);
        robot.intakeLoad.emptied();          // a reading exists, but the estimate holds 4
        mech.pollen().sensed(4);
        mech.volley(4);
        assertEquals(AutoMechanisms.PollenState.SPIN_UP, mech.pollenState());
    }

    @Test
    void offAScoringSpotTheGateStaysShutAndTheVolleyIsSkipped() throws InterruptedException {
        double savedWait = ShootingPose.WAIT_FOR_POSE_S, savedSpin = ScoreCommand.SPINUP_TIMEOUT_S;
        ShootingPose.WAIT_FOR_POSE_S = 0.05;
        ScoreCommand.SPINUP_TIMEOUT_S = 0.01;
        try {
            BioHorsRobot robot = robot();
            AutoMechanisms mech = new AutoMechanisms(robot, 4, () -> false);
            mech.pollen().sensed(4);
            mech.volley(4);
            long deadline = System.nanoTime() + 3_000_000_000L;
            while (!mech.volleyDone() && System.nanoTime() < deadline) {
                mech.update();
                assertTrue(robot.pollenGate.isClosed(), "the gate opened off a scoring spot");
                Thread.sleep(5);
            }
            assertTrue(mech.volleyDone(), "the volley should give up");
            assertEquals(1, mech.volleysSkipped());
            assertEquals(4, mech.pollen().held(), "the pollen stay aboard");
        } finally {
            ShootingPose.WAIT_FOR_POSE_S = savedWait;
            ScoreCommand.SPINUP_TIMEOUT_S = savedSpin;
        }
    }

    @Test
    void knockedOffTheSpotMidVolleyTheGateShutsAndOnlyWhatWasFedCounts() throws InterruptedException {
        double savedWait = ShootingPose.WAIT_FOR_POSE_S, savedSpin = ScoreCommand.SPINUP_TIMEOUT_S;
        ShootingPose.WAIT_FOR_POSE_S = 0.1;
        ScoreCommand.SPINUP_TIMEOUT_S = 0.01;
        try {
            BioHorsRobot robot = robot();
            java.util.concurrent.atomic.AtomicBoolean onSpot = new java.util.concurrent.atomic.AtomicBoolean(true);
            AutoMechanisms mech = new AutoMechanisms(robot, 4, onSpot::get);
            mech.pollen().sensed(4);
            mech.volley(4);
            long deadline = System.nanoTime() + 3_000_000_000L;
            long knockedAt = 0;
            while (!mech.volleyDone() && System.nanoTime() < deadline) {
                mech.update();
                if (knockedAt == 0 && mech.pollenState() == AutoMechanisms.PollenState.FIRING) {
                    Thread.sleep(400);           // about one pollen fed
                    mech.update();
                    onSpot.set(false);           // then bumped off the spot
                    knockedAt = System.nanoTime();
                } else if (knockedAt != 0) {
                    assertTrue(robot.pollenGate.isClosed(), "the gate stayed open off the spot");
                }
                Thread.sleep(5);
            }
            assertTrue(mech.volleyDone(), "the volley should end");
            assertEquals(1, mech.volleysCutShort());
            assertEquals(0, mech.volleysFired(), "a cut-short volley is not a fired one");
            assertTrue(mech.pollen().held() > 0 && mech.pollen().held() < 4,
                    "only what was fed counts: held " + mech.pollen().held());
        } finally {
            ShootingPose.WAIT_FOR_POSE_S = savedWait;
            ScoreCommand.SPINUP_TIMEOUT_S = savedSpin;
        }
    }
}
