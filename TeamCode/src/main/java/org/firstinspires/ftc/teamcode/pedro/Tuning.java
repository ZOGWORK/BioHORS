package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * The Pedro tuners for this robot: Pinpoint odometry and a mecanum drive. Pedro's tuning library
 * finds every static {@link Tuner} method here at startup and lists its procedure on the
 * tuning web page. Run them in this order:
 *
 * <ol>
 *   <li>Mecanum Tuner: which way each drive motor spins. Names are frontLeft, frontRight,
 *       backLeft, backRight. Put the directions in {@link Constants#drivetrainConfig}.</li>
 *   <li>Pinpoint Tuner: pod directions and offsets (4-bar pods, device "pinpoint"). Put them in
 *       {@link Constants#localizerConfig}; the X pod offset there is MEASURED FROM CAD, so
 *       compare.</li>
 *   <li>Foresight Tuner: top speeds, deceleration, heading and braking. Put them in
 *       {@link Constants#foresightConfig()} and the MAX_* / *_DECEL_* constants, which are all
 *       still PLACEHOLDER (SET FROM SIM). Keep autos/robot.json's speeds equal to them.</li>
 *   <li>Tests: follows test paths with the tuned follower to check the result.</li>
 * </ol>
 *
 * The OTOS, OctoQuad, two-wheel and three-wheel tuners in {@code procedures/} are for other
 * localizers; this robot has none of them, so they are not listed.
 */
public final class Tuning {

    private Tuning() {
    }

    @Tuner(name = "1. Mecanum Tuner")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Pinpoint Tuner")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "3. Foresight Tuner")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner(name = "4. Tests")
    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer,
                () -> new Foresight(Constants.foresightConfig()));
    }
}
