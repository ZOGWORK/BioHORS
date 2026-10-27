package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Two gate servos (left + right). Both move together by default; each side
 * can also be set on its own if the gates ever need to move independently.
 *
 * Gate positions are passed in by the OpMode because they depend on how the
 * servos are mounted on THIS robot.
 *
 * TODO: TUNE THESE in the teleop -- GATE_OPEN / GATE_CLOSED there are
 * placeholders. Move the gates by hand in the driver station, read the
 * positions that actually open and close them, and put those numbers in.
 */
public class GateController {

    private final Servo gateLeft;
    private final Servo gateRight;
    private final double gateOpenPos;
    private final double gateClosedPos;

    private boolean gateClosed = true;

    public GateController(Servo gateLeft, Servo gateRight,
                          double gateOpenPos, double gateClosedPos) {
        this.gateLeft = gateLeft;
        this.gateRight = gateRight;
        this.gateOpenPos = gateOpenPos;
        this.gateClosedPos = gateClosedPos;
        setGateClosed(true);
    }

    /** Open or close both gates together. */
    public void setGateClosed(boolean closed) {
        gateClosed = closed;
        double pos = closed ? gateClosedPos : gateOpenPos;
        gateLeft.setPosition(pos);
        gateRight.setPosition(pos);
    }

    /** Flip both gates between open and closed. */
    public void toggleGate() {
        setGateClosed(!gateClosed);
    }

    public void setLeftGateClosed(boolean closed) {
        gateLeft.setPosition(closed ? gateClosedPos : gateOpenPos);
    }

    public void setRightGateClosed(boolean closed) {
        gateRight.setPosition(closed ? gateClosedPos : gateOpenPos);
    }

    public boolean isGateClosed() {
        return gateClosed;
    }
}
