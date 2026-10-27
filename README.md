# BioHORS

FTC robot code for the 2026-27 **BIOBUZZ** season, built on the
**PedroPathing Quickstart v3.0.2** (PedroPathing v3, FTC SDK 12.0.0).
Seeded from HORS-Pedro; the MainHORS control scheme carries over, the old
code does not.

Robot: mecanum drive with Pinpoint odometry (2 pods), 2 intake motors,
**2 independent shooters** (pollen and nectar, one goBILDA 5000-series
6000 RPM motor each, same spin direction, different speeds), **2
independent gate servos** (one per shooter). No turret: aiming turns the
whole robot.

Planned: the MainHORS teleop with odometry aim, and a Zenith-planned 2-tip
pollen auto verified in SimLoop.

## OpModes

| OpMode | File | What it does |
|---|---|---|
| HORS Style TeleOp | `teamcode/teleop/HorsStyleTeleOp.java` | Current teleop (see controls below). Will be replaced by the MainHORS teleop |
| Simple Park Auto | `teamcode/auto/SimpleParkAuto.java` | Encoder move-and-park placeholder, not a Pedro auto. Will be replaced by the Zenith 2-tip auto |
| Tuning (Quickstart) | `teamcode/pedro/Tuning.java` + `procedures/` | PedroPathing tuners, including the Pinpoint Tuner |

## Hardware map (must match the Robot Controller config exactly)

| Name | Device | Notes |
|---|---|---|
| frontLeft, backLeft | DcMotor | Drive, FORWARD |
| frontRight, backRight | DcMotor | Drive, REVERSE |
| intakeLeft, intakeRight | DcMotor | Intakes, run together |
| shooterRight | DcMotorEx | **Pollen** shooter (assumed; confirm on the robot) |
| shooterLeft | DcMotorEx | **Nectar** shooter (assumed; confirm on the robot) |
| gateRight | Servo | **Pollen** gate (confirmed) |
| gateLeft | Servo | **Nectar** gate (confirmed) |
| pinpoint | GoBildaPinpointDriver | Pinpoint odometry computer (2 pods), used by the Pedro localizer |

Shooter/gate names, directions, and tuning values live in
`teamcode/subsystems/MechanismConstants.java`.

## Teleop controls (current; gamepad2 mirrors mechanism buttons)

- Left stick: drive (forward / strafe)
- Right stick X: turn
- Left bumper (hold): slow mode (0.45x)
- Right trigger: intake IN
- Left trigger: intake OUT
- Dpad down: toggle both shooters on/off, each at its own target (start OFF)
- Dpad left / right: **pollen** shooter target RPM -50 / +50
- B: toggle both gates, X: open both gates, A: close both gates
- Gamepads rumble once when either shooter first reaches its target RPM

## Nothing is tuned yet

Every tuning number in the code carries a provenance label: `SPEC` (from a
spec sheet), `PLACEHOLDER` (made up so the code runs), or
`NEEDS MEASUREMENT`. None are measured. Do not treat any as real.

1. **Shooters** (`subsystems/MechanismConstants.java`): each shooter has
   its own target RPM, PIDF, tolerance, and feedforward. Gains are
   placeholders. `ticksPerRev` = 28 and feedforward max = 2800 ticks/sec
   are the goBILDA 6000 RPM motor spec. `ticksPerRev` counts per flywheel
   revolution, so 28 only holds if the flywheel is direct-driven (1:1);
   with a belt/gear ratio, multiply it by the ratio. The 2800 ticks/sec is
   the motor encoder's free speed and does not change with the ratio.
   Shooter motor directions and which motor is pollen still need
   verifying on the robot.
2. **Gates** (`subsystems/MechanismConstants.java`): open/closed position
   per gate. Both start at the old shared placeholders (0.67 / 0.485).
3. **Pinpoint** (`pedro/Constants.java`): pod offsets and directions. Run
   the Quickstart's Pinpoint Tuner on the robot.
4. **Drive shaping, intake power, park auto**: placeholders, labelled in
   their files.

## Building

Open in Android Studio (Narwhal 3 Feature Drop or later) and build like any
FTC SDK project. The PedroPathing dependencies (revhub 3.0.2, tuning 1.0.1)
are in `build.dependencies.gradle`.

## Workflow

Work on branches and open PRs; Greptile reviews every PR. Do not push to
`main` directly.
