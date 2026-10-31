# BioHORS tuning brief

Nothing on this robot is tuned yet. Every number in the code is a placeholder (`PLACEHOLDER`),
a spec-sheet value (`SPEC`), or was chosen so the SimLoop model behaves (`SET FROM SIM`,
`CALIBRATED FROM SIM`). The SimLoop tests pass on those numbers, which proves the code's logic and
wiring, **not** that the real robot will do the same. This brief is the work that makes it real.

Work top to bottom: later items depend on earlier ones (the auto needs the drivetrain, which needs
odometry). After each item, update the value **and its provenance comment** (e.g. `MEASURED
2026-11-02, 5 runs`), then re-run `./gradlew :TeamCode:testDebugUnitTest`.

Paths below are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` unless they start
with `autos/` or `TeamCode/src/test/`.

---

## 0. Confirm the wiring first (no tuning, just look)

| Item | Current | Where | How | Done when |
|---|---|---|---|---|
| Which shooter motor is pollen | `shooterRight` = pollen, `shooterLeft` = nectar, one motor each | `subsystems/MechanismConstants.java` | **Confirmed by the team (Oct 27)**: nectar left, pollen right (top-down, intake up) | Done; check the config names match |
| Shooter directions | both `FORWARD` (placeholder) | `MechanismConstants.POLLEN/NECTAR_SHOOTER_DIRECTION` | Turn each on in the teleop; both must fire pollen/nectar outward, spinning the same physical way | Both launch outward |
| Flywheel gear ratio | **1:1, confirmed by the team (Oct 27)** | `MechanismConstants` (`ticksPerRev = 28`) | — | Done |
| Gate sides | `gateRight` pollen, `gateLeft` nectar (confirmed Oct 27) | `MechanismConstants` | Move each gate from the Driver Station | Matches |
| How pollen reaches the shooter | intake pushes it through the open gate (assumed) | `zenith/ScoreCommand.java` | Watch a volley | If not, change ScoreCommand only |
| Which way the shooter fires | out of the robot's front | `aim/AimController.SHOOTER_FACING_OFFSET_RAD = 0` | Look | `0` front, `Math.PI` back, etc. |
| Pinpoint pod type | goBILDA 4-bar | `pedro/Constants.java` | Look at the pods | Matches |

## 1. Pinpoint odometry

| Item | Current | Where | How | Done when |
|---|---|---|---|---|
| X / Y pod offsets | `0.0`, `0.0` in (PLACEHOLDER) | `pedro/Constants.java` `localizerConfig` | Tuning OpMode, **Pinpoint Tuner** | Spin the robot in place 10 times: pose drifts < 1 in |
| Pod directions | both `FORWARD` (PLACEHOLDER) | same | Pinpoint Tuner | Push forward: x goes up; push left: y goes up |
| Sim copy of the offsets | `0.0`, `0.0` | `TeamCode/src/test/.../sim/BioHorsSim.java` `X/Y_POD_OFFSET_IN` | Copy the measured values | Sim and robot agree |

## 2. Pedro drivetrain (Foresight)

All in `pedro/Constants.java`, all `SET FROM SIM` today. Run **Tuning OpMode -> Foresight Tuner**
(it prints every field below), paste its output, and keep `autos/robot.json` `kinematics` equal.

| Item | Current | Done when |
|---|---|---|
| Max forward / strafe velocity | 60 / 50 in/s | Tuner values in Constants **and** `robot.json` `maxForwardVelInPerS` / `maxStrafeVelInPerS` |
| Natural forward / strafe deceleration | 60 / 50 in/s^2 | Tuner values in both files (`forwardDecelInPerS2` / `strafeDecelInPerS2`) |
| Forward / strafe translational P (primary, secondary) | 0.12 / 0.04, 0.15 / 0.06 | A 48 in line ends within 0.5 in |
| Heading P | 3.0 | A 90 deg turn on the spot settles within 2 deg, no oscillation |
| Coast / brake kV | 0.02 / 0.05 | Tuner values |
| Brake coefficients (linear, quadratic, heading) | example values | Tuner values |
| Heading deviation tolerance | 3 deg | Keep 3 deg unless volleys start facing off the cell |
| `robot.json` `accelInPerS2` | 220 (CALIBRATED FROM SIM) | Re-run `npx zenith calibrate --write` on **robot** traces, not sim traces |
| `robot.json` `maxAngularVelRadPerS` | 5 | Spin in place at full yaw, read the Pinpoint |

## 3. Shooters (use the Shooter Tuner OpMode)

Both shooters are tuned separately; the **Shooter Tuner** OpMode (`tuning/ShooterTuner.java`,
group "Tuning") does the measuring. Do section 0 (gear ratio, directions, which motor is which)
first.

1. Robot on blocks or in a clear area, **no pollen loaded**, a reasonably fresh battery.
2. Run **Shooter Tuner**. It does the pollen shooter, then the nectar shooter, one at a time:
   - **Feedforward:** runs 0.2, 0.4, 0.6, 0.8, 1.0 power, waits for a steady speed, records speed
     and battery voltage, and fits a line. Slope -> `ffReferenceMaxTicksPerSec` (normalised to
     12 V), intercept -> `kS` (static friction). `ShooterController` scales both by
     12 V / measured voltage, so they hold as the battery drains.
   - **P:** with that feedforward and I = D = 0, steps from 70 % to the target RPM and raises P
     until the step overshoots more than 3 % or rings, then backs off 20 %.
   - **Hold check:** spin-up time from rest, steady error band, and recovery after a 0.15 s power
     cut (a simulated pollen passing through).
   - **Together check:** with both tuned, it spins **both shooters at once** at their own targets
     (the teleop's dpad down runs them together, and they share one battery), settles 3 s, and
     measures each one's error over 2 s plus the battery voltage. **Pass:** each within the same
     2 % band. The report shows each shooter's band alone vs together and the battery voltage
     alone vs together.
3. Copy the printed lines (also in the robot log, tag `ShooterTuner`) into each shooter's
   "tuned values" block in `subsystems/MechanismConstants.java`.
4. **Done when** the hold check shows spin-up well under a second, steady within about 2 %, and quick
   recovery; then shoot real pollen and watch it score.

| Item | Current | Who sets it |
|---|---|---|
| `ffReferenceMaxTicksPerSec`, `kS`, `kF = 1` | 2800 (SPEC no-load), 0, 1.0 | Shooter Tuner |
| `kP` | 0.001 (PLACEHOLDER) | Shooter Tuner |
| `kI`, `kD` | 0.0005, 0 (PLACEHOLDER) | Shooter Tuner sets both to 0 (flywheels do not need them) |
| Pollen `targetRpm` | **1600** (the team's test value) | **Field test**, not the tuner: shoot from the auto's volley spots (the start, `gardenTurn`, `scoreNorth`), trim with dpad left/right in the teleop until pollen lands 10 of 10 |
| Nectar `targetRpm` | **1600** (the team's test value; trim with left bumper / A) | Field test, nectar into the flower |
| `ticksPerRev` | 28 (SPEC, 1:1) | Section 0 gear ratio (the tuner works in encoder ticks/s, so it does not need it) |
| `rpmTolerance` | 75 rpm | Shot-to-shot spread vs rpm |

**If the together check fails, do not retune the PID blindly.** The two loops are independent;
the shared battery is the only coupling, so it is almost certainly voltage. Check in this order:

1. **Voltage compensation.** `ShooterController` scales the feedforward and `kS` by
   `ffReferenceVoltage / measured voltage` (unit-tested in `ShooterControllerVoltageTest`). Check the
   voltage sensor reads the real battery (the tuner prints the voltage it saw).
2. **Voltage range.** If the together run was on a low battery (the report says), re-run on a fresh
   one and compare. The values must hold across the voltages a match actually sees (about 12.8 V
   fresh down to about 11.5 V).
3. Only if it still fails with compensation working: re-run the tuner, and write down what changed
   between the separate and together runs.

In the sim (`ShooterTunerSimTest`), with a battery that sags under both shooters and drains from
12.9 V to about 11.6 V, both shooters hold within 0.7 % together. The sim's coupling is mild: at the
P the tuner picks, P alone absorbs a 10 % voltage drop, so the sim cannot show compensation is
*needed*; the real robot is the test of that.

The tuner does not pick target RPM, does not guarantee scoring, and does not touch the
drivetrain (Pedro's follower is a separate loop, section 2).

## 4. Gates

`subsystems/MechanismConstants.java`: `POLLEN_GATE_OPEN/CLOSED`, `NECTAR_GATE_OPEN/CLOSED`, all
`0.67 / 0.485` (PLACEHOLDER, the old shared values). Move each gate from the Driver Station, read
the servo position that just opens / fully closes it. **Done when** each gate opens without
binding and closes without letting a pollen past.

## 4b. Flower bar

Two servos, `flowerLeft` and `flowerRight` (**add them to the Robot Controller config by these
names**), mounted facing away from each other: `FlowerBarController` sets the right one to REVERSE,
so one position number moves both the same way. If the bar twists instead of lifting, the other
servo is the reversed one: swap which gets REVERSE.

`MechanismConstants`: `FLOWER_BAR_STOWED` 0.15, `FLOWER_BAR_DOWN` 0.80, `FLOWER_BAR_SHOOTING` 0.65,
all PLACEHOLDER (arbitrary). Set them from the Driver Station:
- STOWED: folded up, the robot at its shortest (it starts the match like this).
- DOWN: out and low, the bar seated in a flower's retrieval opening.
- SHOOTING: DOWN raised just enough that launched pollen clear it.

Teleop: **X toggles DOWN / STOWED**; while either gate is open the bar sits at SHOOTING. Auto: DOWN
only while collecting from a flower, SHOOTING for every volley. **Done when** the bar seats in a
flower's opening at DOWN, shots never touch it at SHOOTING, and the robot fits the start box at STOWED.

## 4c. Pollen count from the intake encoders (auto only)

Both intake motors have encoders. Pollen aboard press on the rollers, so pulling in at full power
the intake slows down; `IntakeLoadSensor` turns the slowdown into a count (0-4). It is the same idea
as MAINHORS26's `IntakeBallDetector` (DECODE), and uses its numbers: **loaded (full) at or below
2100 ticks/s**, and a reading must hold for 0.15 s. With a free speed of 2800 ticks/s (PLACEHOLDER)
each pollen is 1/4 of the way down to 2100.

In AUTO: collecting stops as soon as it reads full (`hopperFull`); a volley is **skipped** if it
reads empty (as MAINHORS26's auto skipped a shot with no balls); and the state machine stops waiting
at a pickup once, after it arrives, a fresh reading shows nothing new for `COLLECT_NO_PROGRESS_S`
(0.5 s). The Zenith auto's waits run to `hopperFull` or their timeout. Teleop does not use it.

To check on the robot: run the intake in, empty, and read "Intake speed" in the auto's telemetry
for `INTAKE_FREE_TICKS_PER_S`. `INTAKE_SENSOR_ENABLED = false` falls back to the time estimate.

## 4d. Teleop intake jam reverse

`IntakeJamGuard`: while right trigger pulls in, if both intake motors' average speed stays below
`INTAKE_JAM_TICKS_PER_S` (700 ticks/s, PLACEHOLDER: a quarter of the free speed) for
`INTAKE_JAM_DETECT_S` (0.25 s), the intake runs outward for `INTAKE_JAM_REVERSE_S` (0.3 s), then
pulls in again. It waits `INTAKE_SENSE_SETTLE_S` after every start, so the spin-up is not a jam.
Letting go of the trigger ends a reverse at once.

The threshold must stay well below a FULL robot (2100 ticks/s, 4c), or a full hopper gets spat out.
To set it: hold a pollen hard against the rollers, read "Intake speed" in the auto's telemetry, and
put the threshold about halfway between that and 2100. Done when a real jam clears and a full
robot held on the trigger never reverses.

## 4e. Failsafe thresholds

| Item | Current | Where | How | Done when |
|---|---|---|---|---|
| Auto spin-up minimum | 80 % of target at the timeout | `MechanismConstants.SPINUP_MIN_FRACTION` | Film shots at 80 % speed | Shots at that speed still score, or raise it |
| Shooter encoder check | pushed at >= 0.3 power, < 50 rpm for 0.5 s | `SHOOTER_ENCODER_CHECK_POWER`, `_DEAD_RPM`, `_DEAD_S` | Time a cold spin-up to 50 rpm | `_DEAD_S` well above it; no false `ENCODER?` all practice |
| Pinpoint watchdog | 5 loops not READY, or never READY 1 s after PLAY | `auto/PinpointWatchdog` | Unplug the Pinpoint mid-route on blocks (wheels off the floor) | The drive stops, `AUTO HALTED` shows |
| Low battery | under 11.5 V for 1 s | `utilities/LowBatteryWarning` | Note the voltage at which shots start falling short | The warning shows before that |
| Teleop / endgame length | 120 s / 30 s, last warning at 10 s | `utilities/EndgameAlerts` | Check the BIOBUZZ game manual | Matches the manual |
| Auto pose handoff | used if saved < 60 s ago | `aim/PoseHandoff.MAX_POSE_AGE_S` | Time a real auto-to-teleop transition | Comfortably above it |
| Idle flywheel speed | 800 rpm, OFF for now | `MechanismConstants.SHOOTER_IDLE_RPM`, `SHOOTER_IDLE_ENABLED` | Time spin-up from idle vs from rest; watch the battery | Spin-up clearly shorter, battery fine |
| Forgotten gate request | dropped after 3 s idle | `MainHorsTeleOp.REQUEST_IDLE_S` | Drive practice | Never drops a request the driver still wanted |
| Slow loop warning | average over 40 ms | `MainHorsTeleOp.SLOW_LOOP_MS` | Read the Loop line on the robot | Normal loops sit well under it |
| Pose sanity | 6 in past a wall, or a jump over 1.5 x top speed x dt + 3 in | `aim/PoseSanity` | Drive flat out and spin hard in practice | Never trips on real driving |
| Pre-match battery | 12.5 V or more | `utilities/PreMatchCheck.MIN_START_V` | Read fresh batteries | Fresh ones pass, used ones fail |
| Late cutoffs (state machine auto) | leave garden by 20.5 s, skip last volley from 25.2 s | `TwoTipStateMachineAuto.LEAVE_GARDEN_BY_S`, `SKIP_LAST_VOLLEY_AFTER_S` | Check if BIOBUZZ scores parking; time the real route | Set from the real timings |

## 5. Auto timing and the pollen estimate

| Item | Current | Where | How | Done when |
|---|---|---|---|---|
| Feed time per pollen | 0.4 s (+0.2 s) | `zenith/ScoreCommand.FEED_S_PER_PIECE`, `FEED_EXTRA_S` | Film a 4-pollen volley; time gate-open to last pollen out | A volley of 4 always empties the robot |
| `score` estimate | `0.4 + count * 0.4` | `autos/robot.json` commands | Match the measured volley | Zenith's estimate within 10 % |
| Intake time per pollen | 0.35 s | `zenith/PollenEstimate.INTAKE_S_PER_PIECE` | Time 4 pollen from a flower and from the garden | `hopperFull` turns true when it really is full |
| **A pollen sensor** | none | — | Beam break or colour sensor in the hopper | Replace `PollenEstimate` with real counts (biggest auto-reliability win) |
| Spin-up timeout | 2.0 s | `ScoreCommand.SPINUP_TIMEOUT_S`, `BioHorsCommands.SPIN_UP_TIMEOUT_S` | Time a cold spin-up | Timeout = measured + 0.5 s |

## 6. Robot geometry for Zenith

`autos/robot.json` is MEASURED FROM CAD (`33950 Robot.step`, see [ROBOT_CAD.md](ROBOT_CAD.md)):
footprint 22.0 x 17.84 in (flower bar tip 14.2 in ahead of the drive centre, back 7.8 in behind),
height 14.9 in, centre of rotation 3.2 in behind the box's middle, intake mouth 13.2 in wide /
4.6 in deep / 11.9 in forward. Check each with a tape on the real robot (CAD can be out of date) and
re-run `npx zenith validate autos/two-tip.auto.json --json`. The flower bar is STOWED at the start
(team, Oct 27), so the robot starts as its 17.4 in frame; `startIn` stays the 22 in box only because
Zenith has one centre of rotation, and that box puts the back on the wall exactly where the stowed
robot's is. **Done when** the tape agrees with the CAD and the auto still validates with 0 errors.

## 7. Waypoints (measure once on a real field)

`autos/waypoints.json` (RED; BLUE is the point mirror). Put the robot where each one says, read the
Pinpoint pose (after setting the start pose), and replace the number and its provenance.

| Waypoint | Current (x, y, heading) | What it is |
|---|---|---|
| `start` | (-15, -64.2, 90 deg) | The fixed start at the hive that is up at kickoff: back on the south wall. Volley 1 (the preloads) is shot from here, 55 in from the south cell, after a 2 deg turn |
| `gardenApproach` / `Pickup` | (-62.5, -45) / (-62.85, -57.0), -90 deg | Garden line, nose-first south. The robot slides slowly to x = -62.5, then noses in to x = -62.85: the 13.2 in mouth reaches the west-most pollen with 0.25 in to spare, the 17.84 in body stays 0.23 in off the field file's west wall (-72), and the bar stops 0.8 in short of the south wall (a follower overshoots by a few tenths). **Check on the field**: FIRST's CAD puts the walls at +/-70.67, 1.3 in inside the field file's, and the field file's garden pollen (x -70.6, radius 1.4) touch its -72 wall, so at least the pollen and walls move together; measure the lane from the real wall and the real pollen |
| `gardenTurn` | (-50, -44, -90 deg) | Room to turn after the garden. Volley 2 (the garden load, tip 1) is shot from here, 51 in from the south cell |
| `flowerWestApproach` / `Pickup` | (-44, -24) / (-52.4, -24), 180 deg | flower0 (near the garden): the flower bar stops 0.5 in short of the foot |
| `scoreNorth` | (-24, 44, -72 deg) | Volleys 3-4, north cell, facing it |
| `flowerNorthPickup` | (-24, 52.4, 90 deg) | flower1 (near the tipped hive): the bar stops 0.5 in short of the foot |
| `park` | (-56, 36, 180 deg) | Partly in RED's loading zone; the last step of the auto (both runtimes) |

**Flower pickups:** the flower bar is built to slide *into* the retrieval opening, but the field
file's foot is solid, so Zenith (and the sim) stop the bar 0.5 in short. On a real field, drive in
until the bar seats and move both pickups to that pose.

**Done when** each is measured, the auto re-validates with 0 errors, and `zenith deploy` has been re-run.

## 8. Aim system (teleop)

| Item | Current | Where | How | Done when |
|---|---|---|---|---|
| Hive coordinates | RED (-12.75, 0), BLUE (12.75, 0); cells +/-9.264 in y | `aim/FieldTargets.java` (SPEC, game manual / CAD) | Confirm on the real field | Matches |
| Aim gains | kP 1.2, max yaw 0.7, min yaw 0.06, tolerance 2 deg | `aim/AimController.java` | Hold left trigger from 5 poses | Settles in < 1 s without oscillation |
| Turn rate at full yaw | 5.0 rad/s | `AimController.MAX_TURN_RATE_RAD_PER_S` | Spin at full yaw for 2 s, read the Pinpoint | Measured value; aim holds while strafing |
| Default start pose | (-15, -64.2, 90 deg) | `teleop/MainHorsTeleOp.java` | Same as the auto's `start` | Matches |

**Check on the real field:** put the robot at a measured pose (e.g. (-40, -40, 0)), hold the left
trigger, and check with a laser or string that the shooter points at the south cell; drive north
across the field's middle and check it swings to the north cell.

## 9. SimLoop model numbers (so the sim matches the robot)

`TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/`:

| Item | Current | Where | How |
|---|---|---|---|
| Drive motor / wheel | 312 rpm, 537.7 cpr, 104 mm (assumed) | `BioHorsSim` | Read the real parts |
| Drive response | 5 /s | `BioHorsSim.DRIVE_RESPONSE_PER_S` | Fit a recorded straight-line run (SimLoop `fit` package) |
| Track width / wheelbase | 12.96 / 10.39 in (MEASURED FROM CAD) | `BioHorsSim` | Tape |
| Shooter spin-up | 3 /s | `BioHorsSim.SHOOTER_RESPONSE_PER_S` | Log rpm from 0 to target; fit the first-order curve |
| Gate transit | full sweep 0.5 s | `BioHorsSim.GATE` | Film the gate |
| Launch cadence, launch cone, flower rate | 0.30 s, 15 deg, 0.15 s | `BiobuzzMatch` | Film volleys and flower pulls |

**Done when** a real run's RLOG and the sim's overlay in AdvantageScope (same pose and rpm curves
within a few percent). SimLoop's `fit` package can compute the gap.

### 9a. The shot (the most important measurement left)

The sim can fly real balls (`ShotModel` -> SimLoop physics -> the physics hive `BiobuzzHive`). How
far and how high a pollen goes depends on two numbers; the angle is now read from CAD, the speed is not:

| Item | Current | Where | How |
|---|---|---|---|
| Exit angle | **75 deg** (MEASURED FROM CAD: the hood face is 15 deg off vertical, team's Onshape sketch, Oct 30) | `ShotModel.POLLEN_PITCH_MEASURED_DEG` | Confirm on film: one shot side-on at 1750 rpm (phone slow-mo, something level in frame); the angle of the first few frames of flight |
| Exit speed / rim speed | 0.5 (a ball rolling on a fixed hood) | `ShotModel.GRIP_EFFICIENCY` | Same video: speed over the first few frames, divided by `rpm * 2 pi / 60 * 1.42 in` |

What the sim says now (`ShotSweepTest`, lowest rpm that seats a pollen in the up cell, robot on
the hive's centre line, y = distance south of the hive axle; -1 = none up to 6000):

| grip, pitch | y -35 | -40 | -45 | -50 | -55 | -60 |
|---|---|---|---|---|---|---|
| 0.5, 44 deg (an old misreading: the ramp, not the hood) | -1 | -1 | -1 | -1 | -1 | -1 |
| 0.5, 60 deg | -1 | -1 | -1 | 3500 | 3000 | 2900 |
| 0.5, 70 deg | -1 | 2900 | 2700 | 2600 | 2700 | 2700 |
| 0.8, 60 deg | -1 | -1 | 5600 | 2200 | 1900 | 1800 |
| 0.8, 70 deg | 3600 | 1800 | 1700 | 1700 | 1700 | 1700 |
| **0.5, 75 deg (measured angle)** | 2800 | 2600 | 2600 | 2700 | 2800 | 2900 |
| **0.8, 75 deg (measured angle)** | 1800 | 1600 | 1600 | 1700 | 1800 | 1800 |

The robot's centre cannot be south of y = -64.2 facing north (its back is 7.8 in behind it).

Two findings:

1. **A flat shot cannot score**: the cell opening is about 47 in above the exit, and the ball has to
   come down into it. At 40 deg or less nothing scores from anywhere; it starts at about 50 deg and is
   most forgiving at 65-75 deg. The hood's 75 deg is in that band: from the 48 in scoring spots
   1650-1800 rpm seat at a 0.8 ball speed (about 2700 at 0.5). The ball speed is what is left to film.
2. **Where the auto shoots from.** The optimized route shoots volleys 1 and 2 from the start
   (55 in from the south cell) and from `gardenTurn` (51 in), in the 45-50 in band where every
   plausible shot works. Volleys 3 and 4 still go from `scoreNorth`, 36.5 in from the north cell,
   which is short for a shallow shot. **Once the shot is measured, rerun `ShotSweepTest` and move
   `scoreNorth` out if the table says so.**

**Where in the cell to aim a rapid volley** (`PocketAimTest`): aim at the cell's middle. Volleys of 8
were fired at the middle, a side wall, and the front-lip and deep corners, from the centre line and
from an angle. They used perfect shots, then scatter (up to 3 deg / 3 %) plus an 8 % flywheel dip per
shot, with hive walls from very dead (restitution 0.02) to lively (0.5). The middle seated as many or
more every time. Wall aims lost 0.5-1 of 8, and the front-lip corners 2-3. The up cell slopes down
to its inner wall, so every ball that gets in rolls to the same place, and aiming off-centre only
adds the chance of clipping the lip or a side wall. The rpm window matters far more: about 200 rpm
wide, and too fast overshoots the cell. **Worth checking on the real robot:** the sim's cell is a
plain box, so a real cell with a funnel or lip inside could change this. The full sweeps run with
`-Dsim.sweeps=true`.

`PhysicsTipDemoTest` shows the physics working: parked 50 in out at 75 deg and the what-if ball speed, the real
ShooterController and gate fire real balls at 1800 rpm: 7 seat, the 8th arrives as the hive starts to
tip (2.7 s), and 8 spill on the south floor. Open `TeamCode/build/sim/physics-tip-demo.rlog` in AdvantageScope's 3D
Field tab (Field/Robot, Field/Pollen as game pieces, Field/Hive as the hive bar's pose).

## 10. The tip rule

The field file uses the season's tip table again, `[8, 7, 6, 3, 1, 0]`: the pollen needed to tip a
cell, by the nectar in it (Zenith's season package and SimLoop/Zenith docs; DSim-sourced). Each up
cell starts with 3 nectar, so **3 pollen tip it the first time**; after a tip the new up cell is
empty and needs 8. This is what Zenith's starter route (the auto now) is built on: the 4 preloads tip
the hive. The team's earlier "8 pollen per tip, whatever the nectar" rule is the one-line change back
(`rules.tipTable` = `[8]` in `autos/field/biobuzz.field.json`); under it this route never tips.

## 11. Seeing the real robot in AdvantageScope

`advantagescope/Robot_BioHORS/` is an AdvantageScope custom robot built from the team's Onshape
STEP (`33950 Robot.step`): the chassis as `model.glb`; the pollen and nectar flywheels (Rhino
wheel, steel flywheel, hub and shaft) as `model_0.glb` and `model_1.glb`; the intake's four spinning
axles front to back (1st stage roller, 2nd stage roller, transfer roller, gecko wheels) as
`model_2.glb` to `model_5.glb`; and the flower bar (plates, bar and both servo hubs) as
`model_6.glb`. The flywheels spin at the shooters' true rpm, the intake axles turn with the intake
motors (1:1 with a 1150 rpm Yellow Jacket, PLACEHOLDER: the CAD has no belts to read a ratio from),
and the flower bar swings with its servos: the CAD has it DOWN; STOWED is drawn 90 deg up and other
positions in proportion (PLACEHOLDER, drawing only). `MechanismsDemoTest` writes
`TeamCode/build/sim/mechanisms-demo.rlog`: the robot standing still while the bar goes stowed,
down, shooting, stowed and the intake runs, and checks both from the log.

**Install it once (AdvantageScope 26):**
1. In AdvantageScope, **Help > Show Assets Folder**.
2. Copy the whole `advantagescope/Robot_BioHORS` folder into it (the folder, not just its files).
3. Restart AdvantageScope. "BioHORS" now appears in the 3D Field tab's robot model list.

**View a sim run:**
1. Run the tests (`.\gradlew.bat :TeamCode:testDebugUnitTest`), then open
   `TeamCode/build/sim/physics-tip-demo.rlog` (or any match RLOG under `TeamCode/build/sim/`).
2. Open a 3D Field tab and pick the 2026-2027 field.
3. Drag `RealOutputs/Field/Robot` in as a **Robot**, and choose **BioHORS** as its model.
4. Drag `RealOutputs/Field/RobotComponents` onto that robot as **Components**: the flywheels spin at
   the shooters' true rpm. Without it they are drawn still, in place.
5. Add `RealOutputs/Field/Pollen` as game pieces, and `RealOutputs/Field/UpCell` as a **Cone** (not a
   Ghost: a ghost robot there hides the cell and the balls going in).
6. Paths: add `RealOutputs/Auto/PlannedRoute` and `RealOutputs/Auto/DrivenPath` and set each one's type
   to **Trajectory** (dropped in, a Pose2d array becomes a ghost robot at every point). Give the driven
   path another colour (Orange): planned against driven. Both autos log both.

The model's origin is the drive centre on the floor, the same point `Field/Robot` logs
(`docs/ROBOT_CAD.md` has the CAD-to-robot transform). Screws, nuts and other parts under 0.45 in
are left out, the two pollen the CAD shows held are left out (the sim draws the real ones), and the
mesh is clustered to a 3 mm grid (1.5 mm for the flywheels), so it stays about 5 MB. Every mesh
carries vertex normals: AdvantageScope draws a model without them as nothing at all (the robot row
shows "BioHORS" but the field is empty), so both tools below write them.
The flywheel spin direction is an APPROX (the ball rides under the wheel toward the front); it is
drawing only and changes no physics.

**Watch the state machine auto:** run `StateMachineAutoMatchTest` (or the whole suite), then open
`TeamCode/build/sim/state-machine-red.rlog` (or `-blue`). Besides the robot and pollen above, add
`RealOutputs/Auto/PlannedRoute` to the 3D (or 2D) Field and set its type to **Trajectory** to draw the
whole planned route; `RealOutputs/Auto/State` shows which state the auto is in at each moment.


**Hives that tip:** AdvantageScope's field model has fixed hives. `advantagescope/tools/build_hive_assets.py`
(Python with numpy; run it once, and again after AdvantageScope updates its field) builds three local
assets from AdvantageScope's own downloaded 2026-2027 field, straight into its assets folder (they
are AdvantageScope's model, so they are not committed): the field with both hives removed,
"2026-2027 Field (tipping hive)", and each hive alone as a robot model, "Hive (Red)" and
"Hive (Blue)", pivoted on its CAD axle (the bearings, 43.95 in up). Restart AdvantageScope, pick the
tipping-hive field, add `RealOutputs/Field/HiveModel` as a **Robot** and choose your alliance's hive
model. In a physics run it is the physics hive's real angle; in a rule-based run it swings to the
new stop over 4 s (APPROX, DSim) after each tip. `RealOutputs/Field/UpCell` still marks the cell
that is up. (The physics hive turns about 42.77 in, APPROX; the CAD axle is 1.2 in higher.)
AdvantageScope's FTC fields use the Center/Rotated frame (its docs, "Coordinate Systems"): +X to the
right as seen from the red alliance wall, so `AdvantageScopeFrame` turns our field frame by 90 deg;
`AdvantageScopeFrameTest` checks our field file against the field model.

**Rebuild it after a CAD change** (Python with `cadquery-ocp` and `numpy`; loading the STEP takes
a minute or two):

```
python advantagescope/tools/step_to_glb.py "<path>/33950 Robot.step" advantagescope/Robot_BioHORS
python advantagescope/tools/decimate_glb.py advantagescope/Robot_BioHORS/model.glb 3
python advantagescope/tools/decimate_glb.py advantagescope/Robot_BioHORS/model_0.glb 1.5
python advantagescope/tools/decimate_glb.py advantagescope/Robot_BioHORS/model_1.glb 1.5
python advantagescope/tools/decimate_glb.py advantagescope/Robot_BioHORS/model_2.glb 3
python advantagescope/tools/decimate_glb.py advantagescope/Robot_BioHORS/model_3.glb 3
python advantagescope/tools/decimate_glb.py advantagescope/Robot_BioHORS/model_4.glb 3
python advantagescope/tools/decimate_glb.py advantagescope/Robot_BioHORS/model_5.glb 3
python advantagescope/tools/decimate_glb.py advantagescope/Robot_BioHORS/model_6.glb 3
```

`step_to_glb.py` prints every component's joint (each flywheel's axle, each intake axle, the flower
bar's servo axis) in the robot frame. If one moved, put those numbers in `config.json`
(`zeroedPosition` is minus the joint, in metres) and in `BiobuzzMatch`'s `*_AXLE_*_IN`,
`INTAKE_AXLES_IN` and `FLOWER_BAR_PIVOT_*_IN` constants; `AdvantageScopeFrameTest` fails until the
two agree. After any rebuild, copy the folder into AdvantageScope's assets folder again and restart it.
