package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

/**
 * One gate servo, feeding one shooter. The robot has two of these and they
 * are independent: viewed from the top with the intake facing up, the
 * RIGHT gate (gateRight) feeds the pollen shooter and the LEFT gate
 * (gateLeft) feeds the nectar shooter.
 *
 * Open / closed positions are per gate (passed in from MechanismConstants)
 * because the two servos are mounted separately and need not share values.
 * They are read each time the gate is commanded, so a live edit (Panels) takes
 * effect on the next command.
 */
public class GateController {

    private final String label;
    private final Servo servo;
    private final DoubleSupplier openPos;
    private final DoubleSupplier closedPos;

    private boolean closed = true;

    public GateController(String label, Servo servo, double openPos, double closedPos) {
        this(label, servo, () -> openPos, () -> closedPos);
    }

    /** With positions read at each command (a live-tunable constant). */
    public GateController(String label, Servo servo, DoubleSupplier openPos, DoubleSupplier closedPos) {
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
        servo.setPosition(closed ? closedPos.getAsDouble() : openPos.getAsDouble());
    }

    public void toggle() {
        setClosed(!closed);
    }

    public boolean isClosed() {
        return closed;
    }
}
