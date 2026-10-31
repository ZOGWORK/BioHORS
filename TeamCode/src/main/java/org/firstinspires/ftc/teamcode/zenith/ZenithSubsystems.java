package org.firstinspires.ftc.teamcode.zenith;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.BioHorsRobot;

/**
 * The SolversLib subsystems the Zenith auto's commands require. Each wraps one part of
 * {@link BioHorsRobot}; the names match the "requires" lists in autos/robot.json
 * (drive, intake, launcher).
 */
public final class ZenithSubsystems {

    private ZenithSubsystems() {
    }

    /** The drive, as the Pedro follower drives it. Every Zenith path step requires it. */
    public static class Drive extends SubsystemBase {
        private final Follower follower;
        private final ForesightConfig config;

        public Drive(Follower follower, ForesightConfig config) {
            this.follower = follower;
            this.config = config;
        }

        public Follower follower() {
            return follower;
        }

        private org.firstinspires.ftc.teamcode.auto.PinpointWatchdog watchdog;
        private Runnable stopMotors;
        private boolean halted;

        /**
         * Halts the drive for the rest of the auto once the watchdog trips (odometry lost): the
         * follower stops and is no longer updated, and the drive motors are held at zero.
         */
        public void watch(org.firstinspires.ftc.teamcode.auto.PinpointWatchdog watchdog, Runnable stopMotors) {
            this.watchdog = watchdog;
            this.stopMotors = stopMotors;
        }

        /** True once the watchdog has halted the drive. */
        public boolean halted() {
            return halted;
        }

        /** The follower updates once per loop, here: the SolversLib scheduler calls periodic every loop. */
        @Override
        public void periodic() {
            if (watchdog != null && watchdog.update()) {
                if (!halted) {
                    halted = true;
                    follower.stop();
                }
                stopMotors.run();
                return;
            }
            follower.update();
        }

        /** Caps a path at a fraction of the robot's top forward speed while it is followed. */
        public Path atSpeed(Path path, double speedFraction) {
            double f = Math.max(0.05, Math.min(1.0, speedFraction));
            return path.with(config.maxPathSpeed.at(f * Constants.MAX_FORWARD_IN_PER_S));
        }
    }

    /** Both intake motors, plus the pollen estimate's intake side (collection only, not feeding). */
    public static class Intake extends SubsystemBase {
        private final BioHorsRobot robot;
        private final PollenEstimate pollen;
        private final ElapsedTime loop = new ElapsedTime();
        private boolean runningIn = false;

        public Intake(BioHorsRobot robot, PollenEstimate pollen) {
            this.robot = robot;
            this.pollen = pollen;
        }

        public void in() {
            runningIn = true;
            robot.intake.intake();
        }

        /**
         * Runs the intake inward to push pollen through the open gate into the shooter. This is
         * not collecting, so it does not add to the pollen estimate (ScoreCommand takes the
         * launched pollen away when the volley ends).
         */
        public void feed() {
            runningIn = false;
            robot.intake.intake();
        }

        public void stop() {
            runningIn = false;
            robot.intake.stop();
        }

        @Override
        public void periodic() {
            double dt = loop.seconds();
            loop.reset();
            if (!runningIn) return;
            if (org.firstinspires.ftc.teamcode.subsystems.MechanismConstants.INTAKE_SENSOR_ENABLED) {
                if (robot.intakeLoad.hasReading()) pollen.sensed(robot.intakeLoad.count());
            } else if (dt < 0.25) {
                pollen.intakeRan(dt);
            }
        }
    }

    /** The flower bar (flowerLeft + flowerRight): stowed, or down to collect from a flower. */
    public static class FlowerBar extends SubsystemBase {
        private final BioHorsRobot robot;

        public FlowerBar(BioHorsRobot robot) {
            this.robot = robot;
            robot.flowerBar.setDown(false);
        }

        public void down() {
            robot.flowerBar.setDown(true);
        }

        public void stow() {
            robot.flowerBar.setDown(false);
        }
    }

    /** The pollen shooter and the pollen gate (gateRight). Auto is pollen only. */
    public static class Launcher extends SubsystemBase {
        private final BioHorsRobot robot;

        public Launcher(BioHorsRobot robot) {
            this.robot = robot;
            robot.pollenGate.setClosed(true);
        }

        public void spinUp() {
            // The auto's speed for its scoring spots (MechanismConstants.AUTO_POLLEN_RPM), not teleop's.
            robot.pollenShooter.setTargetRpm(org.firstinspires.ftc.teamcode.subsystems.MechanismConstants.AUTO_POLLEN_RPM);
            robot.pollenShooter.setShooterOn(true);
        }

        public void stop() {
            robot.pollenShooter.setShooterOn(false);
        }

        public boolean atSpeed() {
            return robot.pollenShooter.isShooterOn() && robot.pollenShooter.isAtTarget();
        }

        /** Fast enough to fire after a spin-up timeout (MechanismConstants.SPINUP_MIN_FRACTION). */
        public boolean nearSpeed() {
            return robot.pollenShooter.atLeastFractionOfTarget(
                    org.firstinspires.ftc.teamcode.subsystems.MechanismConstants.SPINUP_MIN_FRACTION);
        }

        private int spinUpFailures;

        /** A volley gave up: the flywheel never got near its speed. */
        public void spinUpFailed() {
            spinUpFailures++;
        }

        /** Volleys skipped this run because the flywheel never got near its speed. */
        public int spinUpFailures() {
            return spinUpFailures;
        }

        public void openGate() {
            robot.pollenGate.setClosed(false);
        }

        public void closeGate() {
            robot.pollenGate.setClosed(true);
        }

        /** The volley emptied the robot: the intake sensor's count starts again from nothing. */
        public void emptied() {
            robot.intakeLoad.emptied();
        }

        /** Lifts the flower bar slightly while a volley runs, so the shots clear it. */
        public void barClearOfShots(boolean shooting) {
            robot.flowerBar.setShooting(shooting);
        }

        /** Both shooters' velocity loops run every loop (the nectar one stays off in auto). */
        @Override
        public void periodic() {
            robot.updateMechanisms();
        }
    }
}
