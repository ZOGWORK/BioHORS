package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * The flower bar: the low bar at the front that drops into a flower's retrieval opening so the
 * intake can pull pollen out of it. Two servos drive it, mounted facing away from each other, so
 * one is set to REVERSE: the same position then turns both the same way and the bar stays level.
 *
 * Positions (the left servo's number; the right one gets 1 - it, MechanismConstants):
 * <ul>
 *   <li>START: both servos centred, at init, until the first command.</li>
 *   <li>STOWED: the bar up. The robot is at its shortest.</li>
 *   <li>DOWN: out and low, for collecting from a flower. The robot is at its longest.</li>
 *   <li>SHOOTING: raised slightly from DOWN while launching, so the shots clear it.</li>
 * </ul>
 * Toggling moves between STOWED and DOWN. Shooting is an override on top: while it is set the bar
 * sits at SHOOTING, and when it is cleared the bar goes back to where it was.
 */
public class FlowerBarController {

    public enum Position { STOWED, DOWN, SHOOTING }

    private final Servo left;
    private final Servo right;
    private final double stowedPos;
    private final double downPos;
    private final double shootingPos;

    private boolean down;
    private boolean shooting;

    public FlowerBarController(Servo left, Servo right, double stowedPos, double downPos, double shootingPos) {
        this.left = left;
        this.right = right;
        this.stowedPos = stowedPos;
        this.downPos = downPos;
        this.shootingPos = shootingPos;
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);  // mirrored mount
        left.setPosition(MechanismConstants.FLOWER_BAR_START);
        right.setPosition(MechanismConstants.FLOWER_BAR_START);
    }

    public void setDown(boolean down) {
        this.down = down;
        apply();
    }

    public void toggle() {
        setDown(!down);
    }

    /** While true the bar sits at SHOOTING; false puts it back where it was. */
    public void setShooting(boolean shooting) {
        if (this.shooting == shooting) return;
        this.shooting = shooting;
        apply();
    }

    public boolean isDown() {
        return down;
    }

    public Position position() {
        return shooting ? Position.SHOOTING : down ? Position.DOWN : Position.STOWED;
    }

    private void apply() {
        double p = shooting ? shootingPos : down ? downPos : stowedPos;
        left.setPosition(p);
        right.setPosition(p);
    }
}
