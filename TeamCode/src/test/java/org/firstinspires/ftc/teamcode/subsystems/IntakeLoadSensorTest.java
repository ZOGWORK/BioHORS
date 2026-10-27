package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.SimElapsedTime;
import org.junit.jupiter.api.Test;

/**
 * IntakeLoadSensor against two fake intake motors: the count follows the load, the start-up
 * transient is not read as pollen, and nothing is read while the intake is off.
 */
class IntakeLoadSensorTest {

    static final double TICK = 0.02;

    /** Runs the intake inward for seconds with `pieces` pollen loading it, and returns the count. */
    private final FakeTimer clock = new FakeTimer();

    int countWith(int pieces, double seconds) {
        FakeMotor l = new FakeMotor(2800.0, 8.0), r = new FakeMotor(2800.0, 8.0);
        IntakeLoadSensor sensor = new IntakeLoadSensor(l, r, SimElapsedTime.factory(clock));
        return run(sensor, l, r, pieces, 1.0, seconds);
    }

    int run(IntakeLoadSensor sensor, FakeMotor l, FakeMotor r, int pieces, double power, double seconds) {
        l.setPower(power);
        r.setPower(power);
        l.setExternalTorqueFraction(-0.06 * pieces);
        r.setExternalTorqueFraction(-0.06 * pieces);
        for (double t = 0; t < seconds; t += TICK) {
            l.update(TICK);
            r.update(TICK);
            clock.advance(TICK);
            sensor.update(power);
        }
        return sensor.count();
    }

    @Test
    void theCountFollowsThePollenAboard() {
        for (int n = 0; n <= 4; n++) assertEquals(n, countWith(n, 1.5), n + " pollen aboard");
    }

    @Test
    void theStartUpIsNotReadAsPollen() {
        assertEquals(-1, countWith(0, 0.2), "no reading during the first moments after starting");
    }

    @Test
    void nothingIsReadWhileTheIntakeIsOff() {
        FakeMotor l = new FakeMotor(2800.0, 8.0), r = new FakeMotor(2800.0, 8.0);
        IntakeLoadSensor sensor = new IntakeLoadSensor(l, r, SimElapsedTime.factory(clock));
        run(sensor, l, r, 3, 0.0, 1.5);
        assertFalse(sensor.hasReading());
    }
}
