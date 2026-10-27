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

The MainHORS teleop with odometry aim, and a Zenith-planned 2-tip pollen auto, both tested in SimLoop.

## OpModes

| OpMode | File | What it does |
|---|---|---|
| MainHORS TeleOp | `teamcode/teleop/MainHorsTeleOp.java` | The teleop: MainHORS controls plus odometry aim (see below) |
| Two tip | `teamcode/zenith/generated/TwoTipAuto.java` (generated) | The 2-tip pollen auto, played from `autos/two-tip.auto.json` by Zenith (see below) |
| Choose Alliance | `teamcode/opmodes/ChooseAlliance.java` | Run before the auto: dpad left = RED, dpad right = BLUE |
| Shooter Tuner | `teamcode/tuning/ShooterTuner.java` | Measures each shooter's feedforward (power sweep) and P (search), prints PIDF to paste into `MechanismConstants`. Robot on blocks, no pollen |
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

## The 2-tip auto (Zenith)

`autos/two-tip.auto.json`, planned with [Zenith](https://zenith.horizon36596.org) and played on the
robot by Zenith's runtime through the Pedro v3 follower and the named commands in
`zenith/BioHorsCommands.java`. Pollen only, 16 pollen in 4 volleys of 4, 8 per tip (team rule):

1. Score the 4 preload into the south cell (the cell up at kickoff for RED).
2. Sweep the RED garden (4), score again: south cell 8, **tip 1**; the north cell comes up.
3. Take flower0's 4 (west wall), go round the west side of the hive frame, score into the north cell.
4. Take flower1's 4 (north wall), score again: north cell 8, **tip 2**. Park in the loading zone.

![The two-tip auto](docs/two-tip.png)

One file for both alliances: written for RED, mirrored for BLUE (BIOBUZZ is point-symmetric). Run
**Choose Alliance** before INIT. When the auto stops, its alliance and pose go to the teleop's aim.

Editing it: `npm install` (pins the Zenith CLI and MCP in `package.json`), then use the Zenith MCP
server (`.mcp.json`) or `npx zenith validate|estimate|render|deploy|sim ... --json`. The
`build-auto` skill (`.claude/skills/build-auto/`, with `references/robot.md` for this robot) is the
workflow. Never edit `TeamCode/src/main/assets/autos/` or `zenith/generated/`: `npx zenith deploy`
writes them. No mid-path markers: Zenith v0.1.2's marker runner needs a Pedro method Pedro 3.0.2 lacks.

## Simulation (SimLoop)

`./gradlew :TeamCode:testDebugUnitTest` (Windows: `.\gradlew.bat ...`) runs the real OpModes on a
laptop against fake hardware, in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/`:

- `BioHorsSim`: this robot's fake devices and physics (drive, Pinpoint, shooters, gates, intake).
- `MainHorsTeleOpSimTest`: the teleop (drive, slow mode, aim, goal switch, aim while strafing,
  Pinpoint-fault fallback, BLUE, shooters, gates, intake, determinism).
- `ZenithAutoMatchTest` + `BiobuzzMatch` + `TracedZenithAuto`: the deployed auto as a BIOBUZZ match,
  stopped at 30 s like the field: RED twice (identical logs) and BLUE once. It checks 16 pollen and
  2 tips in AUTO, every step in order, no structure contact, strafing under 20 % of the driving,
  and the robot's belief against the sim's truth. `npx zenith sim two-tip` runs the same test.

Every sim number is a placeholder until measured (`docs/TUNING.md`), so the sim proves the code's
logic and wiring, not the real robot's timing.

RLOGs and Zenith traces land in `TeamCode/build/sim/` (gitignored; re-run the tests to regenerate).
Open them in [AdvantageScope](https://docs.advantagescope.org/): `two-tip-red.rlog` (plot
`Truth/xIn` vs `Truth/yIn` on the 2D field, `Match/scored`, `Match/tips`, `Shooter/pollenRpm`) and
`teleop-aim.rlog` (`Truth/headingRad`).

## Nothing is tuned yet

The full brief, item by item, is [`docs/TUNING.md`](docs/TUNING.md).

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
5. **Drive shaping, intake power, Foresight gains, auto timing**: placeholders, labelled in
   their files.

## Building

Open in Android Studio (Narwhal 3 Feature Drop or later) and build like any
FTC SDK project. The PedroPathing dependencies (revhub 3.0.2, tuning 1.0.1)
are in `build.dependencies.gradle`.

## Horizon's tools

Zenith (beta) and SimLoop (alpha) are by Horizon (FTC 36596) and are early; they are looking for
testers and contributors. Zenith: <https://zenith.horizon36596.org> · SimLoop:
<https://github.com/Horizon-36596/simloop> · Example robot:
<https://github.com/Horizon-36596/Horizon-Library-Examples> · Docs: <https://libraries.horizon36596.org> ·
Discord: <https://discord.gg/p2D3JpM4f6> · Release video: <https://youtu.be/NFww9Mt-ZR4>

## Workflow

Work on branches and open PRs; Greptile reviews every PR. Do not push to
`main` directly.
