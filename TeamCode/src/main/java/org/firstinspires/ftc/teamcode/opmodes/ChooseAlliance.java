package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aim.Alliance;
import org.firstinspires.ftc.teamcode.aim.PoseHandoff;

/**
 * Run this before the auto: dpad left = RED, dpad right = BLUE. The choice is kept on the SDK
 * blackboard until the robot restarts, and the Zenith auto mirrors itself to match.
 */
@TeleOp(name = "Choose Alliance", group = "Setup")
public class ChooseAlliance extends LinearOpMode {

    @Override
    public void runOpMode() {
        Alliance alliance = PoseHandoff.savedAlliance();
        if (alliance == null) alliance = Alliance.RED;
        while (!isStopRequested()) {
            if (gamepad1.dpad_left) alliance = Alliance.RED;
            if (gamepad1.dpad_right) alliance = Alliance.BLUE;
            PoseHandoff.saveAlliance(alliance);
            telemetry.addData("Alliance", "%s  (dpad left = RED, dpad right = BLUE)", alliance);
            telemetry.addData("", "Stop this OpMode, then INIT the auto.");
            telemetry.update();
            sleep(20);
        }
    }
}
