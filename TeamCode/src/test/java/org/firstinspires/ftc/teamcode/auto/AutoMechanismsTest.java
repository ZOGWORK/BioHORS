package org.firstinspires.ftc.teamcode.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;
import org.firstinspires.ftc.teamcode.sim.BioHorsSim;
import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.horizon36596.simloop.opmode.RecordingTelemetry;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * A volley is skipped only when the intake sensor has read the robot empty (as MAINHORS26's auto
 * skipped a shot with no balls). Not knowing, or the sensor turned off, still shoots.
 */
class AutoMechanismsTest {

    private final boolean savedEnabled = MechanismConstants.INTAKE_SENSOR_ENABLED;

    @AfterEach
    void restore() {
        MechanismConstants.INTAKE_SENSOR_ENABLED = savedEnabled;
    }

    static BioHorsRobot robot() {
        return new BioHorsRobot(new BioHorsSim().hardwareMap, new RecordingTelemetry());
    }

    @Test
    void aVolleyIsSkippedWhenTheSensorReadsEmpty() {
        BioHorsRobot robot = robot();
        AutoMechanisms mech = new AutoMechanisms(robot, 0);
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
        AutoMechanisms mech = new AutoMechanisms(robot, 0);
        assertFalse(robot.intakeLoad.hasReading());
        mech.volley(4);
        assertEquals(AutoMechanisms.PollenState.SPIN_UP, mech.pollenState());
        assertEquals(0, mech.volleysSkipped());
    }

    @Test
    void withTheSensorOffItStillShoots() {
        MechanismConstants.INTAKE_SENSOR_ENABLED = false;
        BioHorsRobot robot = robot();
        AutoMechanisms mech = new AutoMechanisms(robot, 0);
        robot.intakeLoad.emptied();
        mech.volley(4);
        assertEquals(AutoMechanisms.PollenState.SPIN_UP, mech.pollenState());
        assertEquals(0, mech.volleysSkipped());
    }

    @Test
    void aLoadedRobotShoots() {
        BioHorsRobot robot = robot();
        AutoMechanisms mech = new AutoMechanisms(robot, 4);
        robot.intakeLoad.emptied();          // a reading exists, but the estimate holds 4
        mech.pollen().sensed(4);
        mech.volley(4);
        assertEquals(AutoMechanisms.PollenState.SPIN_UP, mech.pollenState());
    }
}
