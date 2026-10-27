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
| MainHORS TeleOp | `teamcode/teleop/MainHorsTeleOp.java` | The teleop: MainHORS controls plus odometry aim (see below) |
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

## Teleop controls (MainHORS TeleOp; gamepad2 mirrors the mechanism buttons)

| Control | Action |
|---|---|
| Left stick | Drive (forward / strafe) |
| Right stick X | Turn (overridden while aiming) |
| Left trigger (hold) | **Aim**: turn the robot to face the selected cell of our hive |
| A (press) | **Switch goal**: the other cell (north/south) of our hive, after a tip |
| Right bumper (hold) | Slow mode, 0.45x |
| Right trigger | Intake in |
| Y (hold) | Outtake |
| Dpad down | Toggle both shooters on/off, each at its own target (start OFF) |
| Dpad left / right | **Pollen** shooter target RPM -50 / +50 |
| B | Toggle both gates (start CLOSED) |
| X, left bumper | Unmapped |

Before PLAY, pick the alliance: dpad left = RED, dpad right = BLUE. If an auto ran first, its
alliance and final pose are picked up automatically (the SDK blackboard).

**Aim.** No camera and no turret: holding the left trigger turns the whole chassis to face the
selected hive cell, from the Pinpoint pose and the hive's fixed field coordinates (BIOBUZZ field
file: RED hive pivot (-12.75, 0), BLUE (+12.75, 0), cell centres 9.264 in either side along y).
The driver keeps forward/strafe; yaw gets priority over translation, and a feedforward on the
bearing rate keeps the aim on target while strafing. The robot does not detect tips: the driver
presses A when the hive tips. If the Pinpoint is not READY or the pose is not a number, aim falls
back to manual turn and telemetry shows `POSE INVALID`. Telemetry always shows the alliance,
target cell, aiming on/off, heading error and pose.

## Simulation (SimLoop)

`./gradlew :TeamCode:testDebugUnitTest` (on Windows `.\gradlew.bat ...`) runs the real teleop
OpMode on a laptop against fake hardware: `TeamCode/src/test/.../sim/BioHorsSim.java` (the robot's
fake devices and physics) and `MainHorsTeleOpSimTest.java` (drive, slow mode, aim, goal switch,
aim while strafing, Pinpoint-fault fallback, BLUE, shooters, gates, intake, determinism). Every
sim number is a placeholder until measured (see the tuning section), so the sim checks the code's
logic and wiring, not the real robot's timing.

RLOGs land in `TeamCode/build/sim/` (gitignored; re-run the tests to regenerate). Open
`teleop-aim.rlog` in AdvantageScope and plot `Truth/headingRad`, `Truth/xIn`, `Truth/yIn` and
`Shooter/pollenRpm`.

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
4. **Aim** (`aim/AimController.java`): kP, yaw cap, minimum yaw, on-target
   tolerance, turn rate at full yaw, and which way the shooter fires
   (`SHOOTER_FACING_OFFSET_RAD`). Teleop default start pose
   (`teleop/MainHorsTeleOp.java`) when no auto ran first.
5. **Drive shaping, intake power, park auto**: placeholders, labelled in
   their files.

## Building

Open in Android Studio (Narwhal 3 Feature Drop or later) and build like any
FTC SDK project. The PedroPathing dependencies (revhub 3.0.2, tuning 1.0.1)
are in `build.dependencies.gradle`.

## Workflow

Work on branches and open PRs; Greptile reviews every PR. Do not push to
`main` directly.
