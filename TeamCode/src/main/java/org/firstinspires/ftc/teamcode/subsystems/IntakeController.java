package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Two intake motors. Both run together at the same power.
 *
 * NOTE: if the two intakes need to spin opposite directions to pull in
 * (mirrored mounting), reverse ONE of them in the OpMode with
 * setDirection(REVERSE) instead of changing powers here.
 */
public class IntakeController {

    private final DcMotor intakeLeft;
    private final DcMotor intakeRight;

    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -1.0;

    public IntakeController(DcMotor intakeLeft, DcMotor intakeRight) {
        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;
    }

    /** Pull in. */
    public void intake() {
        setPower(INTAKE_POWER);
    }

    /** Push out (clear a jam / eject). */
    public void outtake() {
        setPower(OUTTAKE_POWER);
    }

    /** Stop both intakes. */
    public void stop() {
        setPower(0.0);
    }

    public void setPower(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }
}
