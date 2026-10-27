package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.junit.jupiter.api.Test;

/**
 * The together check's first question: is the feedforward (and kS) scaled by nominal voltage over
 * measured voltage? With feedback off, the power commanded on a 10 V battery must be exactly
 * 12 / 10 times the power on a 12 V one.
 */
class ShooterControllerVoltageTest {

    private static double feedforwardPower(double volts) {
        ShooterController.Config c = new ShooterController.Config();
        c.label = "test";
        c.ticksPerRev = 28;
        c.kP = 0;
        c.kI = 0;
        c.kD = 0;
        c.kF = 1.0;
        c.kS = 0.05;
        c.integralLimit = 0;
        c.derivativeAlpha = 0;
        c.rpmFilterAlpha = 1.0;
        c.powerSmoothingAlpha = 1.0; // no smoothing: one update gives the commanded power
        c.targetRpm = 2000;
        c.rpmTolerance = 50;
        c.ffReferenceVoltage = 12.0;
        c.ffReferenceMaxTicksPerSec = 2800;
        FakeMotor motor = new FakeMotor(2800, 3.0, 28);
        ShooterController s = new ShooterController(c, motor, null, new FakeVoltageSensor(volts));
        s.setShooterOn(true);
        s.update();
        return motor.getPower();
    }

    @Test
    void feedforwardAndStaticFrictionScaleWithTwelveOverMeasuredVolts() {
        double at12 = feedforwardPower(12.0);
        assertEquals(2000 * 28 / 60.0 / 2800 + 0.05, at12, 1e-9, "at 12 V: speed fraction plus kS");
        assertEquals(at12 * 12.0 / 10.0, feedforwardPower(10.0), 1e-9, "a 10 V battery needs 12/10 the power");
        assertEquals(at12 * 12.0 / 13.0, feedforwardPower(13.0), 1e-9, "a 13 V battery needs 12/13 the power");
    }
}
