package org.firstinspires.ftc.teamcode.tuning.autotune;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.MechanismConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;

/** FtcAutoTune on the pollen shooter. See {@link ShooterAutoTuneOpMode}. */
@TeleOp(name = "AutoTune Pollen Shooter", group = "Tuning")
public class PollenShooterAutoTune extends ShooterAutoTuneOpMode {

    @Override
    String label() {
        return "Pollen";
    }

    @Override
    String field() {
        return "POLLEN_SHOOTER";
    }

    @Override
    String motorName() {
        return MechanismConstants.POLLEN_SHOOTER_NAME;
    }

    @Override
    DcMotorSimple.Direction direction() {
        return MechanismConstants.POLLEN_SHOOTER_DIRECTION;
    }

    @Override
    ShooterController.Config config() {
        return MechanismConstants.POLLEN_SHOOTER;
    }
}
