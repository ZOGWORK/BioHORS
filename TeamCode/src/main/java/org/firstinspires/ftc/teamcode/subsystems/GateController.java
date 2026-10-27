package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * One gate servo, feeding one shooter. The robot has two of these and they
 * are independent: viewed from the top with the intake facing up, the
 * RIGHT gate (gateRight) feeds the pollen shooter and the LEFT gate
 * (gateLeft) feeds the nectar shooter.
 *
 * Open / closed positions are per gate (passed in from MechanismConstants)
 * because the two servos are mounted separately and need not share values.
 */
public class GateController {

    private final String label;
    private final Servo servo;
    private final double openPos;
    private final double closedPos;

    private boolean closed = true;

    public GateController(String label, Servo servo, double openPos, double closedPos) {
        this.label = label;
        this.servo = servo;
        this.openPos = openPos;
        this.closedPos = closedPos;
        setClosed(true);
    }

    public String getLabel() {
        return label;
    }

    public void setClosed(boolean closed) {
        this.closed = closed;
        servo.setPosition(closed ? closedPos : openPos);
    }

    public void toggle() {
        setClosed(!closed);
    }

    public boolean isClosed() {
        return closed;
    }
}
