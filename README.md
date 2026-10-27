# HORS-Pedro

HORS-style FTC robot code, built on the **PedroPathing Quickstart v3.0.2**
(PedroPathing v3). Private repo, modeled on MAINHORS26's teleop structure
but for a different robot: **no turret**, 4 mecanum drive motors, 2 intake
motors, 2 encoder flywheel (shooter) motors, 2 gate servos.

## OpModes

| OpMode | File | What it does |
|---|---|---|
| HORS Style TeleOp | `teamcode/teleop/HorsStyleTeleOp.java` | Main teleop (see controls below) |
| Simple Park Auto | `teamcode/auto/SimpleParkAuto.java` | Drives forward a short distance, then parks. Does NOT use PedroPathing on purpose -- it is just move-and-park |
| Tuning (Quickstart) | `teamcode/pedro/Tuning.java` + `procedures/` | PedroPathing tuners, including the Pinpoint Tuner -- run these on the robot before building path autos |

## Hardware map (must match the Robot Controller config exactly)

| Name | Device | Notes |
|---|---|---|
| frontLeft, backLeft | DcMotor | Drive, FORWARD |
| frontRight, backRight | DcMotor | Drive, REVERSE |
| intakeLeft, intakeRight | DcMotor | Intake pair |
| shooterLeft, shooterRight | DcMotorEx | Flywheel pair, encoders on, left REVERSE / right FORWARD |
| gateLeft, gateRight | Servo | Gate pair |
| pinpoint | GoBildaPinpointDriver | Pinpoint odometry computer (2 pods) -- used by the PedroPathing localizer, not by teleop/auto yet |

## Teleop controls (gamepad1; gamepad2 mirrors mechanism buttons)

- Left stick: drive (forward / strafe)
- Right stick X: turn
- Left bumper (hold): slow mode (0.45x)
- Right trigger: intake IN
- Left trigger: intake OUT
- Dpad down: toggle shooter on/off (shooter starts OFF)
- Dpad left / right: shooter target RPM -50 / +50
- B: toggle gates, X: open gates, A: close gates
- Gamepads rumble once when the flywheel first reaches its target RPM

## Before a match: tune these (all marked TODO in code)

1. **Flywheel PIDF** (`subsystems/FlywheelController.java`): kP, kI, kD, kF,
   TARGET_RPM, RPM_TOLERANCE, TICKS_PER_REV, and the feedforward max
   ticks/sec. The values in the file are PLACEHOLDERS and are intentionally
   NOT the HORS values. Retune them on this robot.
2. **Gate positions** (`teleop/HorsStyleTeleOp.java`): GATE_OPEN / GATE_CLOSED
   are placeholders. Read the real positions off the driver station.
3. **Park auto** (`auto/SimpleParkAuto.java`): DRIVE_TICKS, DRIVE_POWER,
   TIMEOUT_SECONDS are small placeholder values. Measure the real distance
   and set them.
4. **Pinpoint** (`pedro/Constants.java`): pod type (4-bar set here; change to
   swingarm if that is what you have), X/Y pod offsets in inches, and pod
   directions. Run the Quickstart's Pinpoint Tuner on the robot to get them.
   The follower itself is wired up (localizer + mecanum drivetrain) but NOT
   used by teleop or the park auto yet -- it is ready for when you build
   real PedroPathing path autos. The follower's Foresight (algorithm) is
   null for now; add one when you write your first path auto.

## Building

Open in Android Studio (Narwhal 3 Feature Drop or later) and build like any
FTC SDK project. The PedroPathing dependencies (revhub 3.0.2, tuning 1.0.1)
are already in `build.dependencies.gradle`.
