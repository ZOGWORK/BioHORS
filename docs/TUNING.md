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
| Which shooter motor is pollen | `shooterRight` = pollen (assumed) | `subsystems/MechanismConstants.java` `POLLEN_SHOOTER_NAME` | Spin each shooter alone from the Driver Station; see which one sits behind the right (pollen) gate | Names confirmed or swapped |
| Shooter directions | both `FORWARD` (placeholder) | `MechanismConstants.POLLEN/NECTAR_SHOOTER_DIRECTION` | Turn each on in the teleop; both must fire pollen/nectar outward, spinning the same physical way | Both launch outward |
| Flywheel gear ratio | 1:1 assumed | `MechanismConstants` (`ticksPerRev = 28`) | Count belt/gear teeth between motor and flywheel | `ticksPerRev = 28 x ratio` (the 2800 ticks/s feedforward max does **not** change) |
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
3. Copy the printed lines (also in the robot log, tag `ShooterTuner`) into each shooter's
   "tuned values" block in `subsystems/MechanismConstants.java`.
4. **Done when** the hold check shows spin-up well under a second, steady within about 2 %, and quick
   recovery; then shoot real pollen and watch it score.

| Item | Current | Who sets it |
|---|---|---|
| `ffReferenceMaxTicksPerSec`, `kS`, `kF = 1` | 2800 (SPEC no-load), 0, 1.0 | Shooter Tuner |
| `kP` | 0.001 (PLACEHOLDER) | Shooter Tuner |
| `kI`, `kD` | 0.0005, 0 (PLACEHOLDER) | Shooter Tuner sets both to 0 (flywheels do not need them) |
| Pollen `targetRpm` | 2000 (PLACEHOLDER) | **Field test**, not the tuner: shoot from `scoreSouth` / `scoreNorth`, trim with dpad left/right in the teleop until pollen lands 10 of 10 |
| Nectar `targetRpm` | 2000 (PLACEHOLDER, must differ) | Field test, nectar into the flower |
| `ticksPerRev` | 28 (SPEC, 1:1) | Section 0 gear ratio (the tuner works in encoder ticks/s, so it does not need it) |
| `rpmTolerance` | 75 rpm | Shot-to-shot spread vs rpm |

The tuner does not pick target RPM, does not guarantee scoring, and does not touch the
drivetrain (Pedro's follower is a separate loop, section 2).

## 4. Gates

`subsystems/MechanismConstants.java`: `POLLEN_GATE_OPEN/CLOSED`, `NECTAR_GATE_OPEN/CLOSED`, all
`0.67 / 0.485` (PLACEHOLDER, the old shared values). Move each gate from the Driver Station, read
the servo position that just opens / fully closes it. **Done when** each gate opens without
binding and closes without letting a pollen past.

## 5. Auto timing and the pollen estimate

| Item | Current | Where | How | Done when |
|---|---|---|---|---|
| Feed time per pollen | 0.4 s (+0.2 s) | `zenith/ScoreCommand.FEED_S_PER_PIECE`, `FEED_EXTRA_S` | Film a 4-pollen volley; time gate-open to last pollen out | A volley of 4 always empties the robot |
| `score` estimate | `0.4 + count * 0.4` | `autos/robot.json` commands | Match the measured volley | Zenith's estimate within 10 % |
| Intake time per pollen | 0.35 s | `zenith/PollenEstimate.INTAKE_S_PER_PIECE` | Time 4 pollen from a flower and from the garden | `hopperFull` turns true when it really is full |
| **A pollen sensor** | none | — | Beam break or colour sensor in the hopper | Replace `PollenEstimate` with real counts (biggest auto-reliability win) |
| Spin-up timeout | 2.0 s | `ScoreCommand.SPINUP_TIMEOUT_S`, `BioHorsCommands.SPIN_UP_TIMEOUT_S` | Time a cold spin-up | Timeout = measured + 0.5 s |

## 6. Robot geometry for Zenith

`autos/robot.json` (all NEEDS MEASUREMENT): footprint 18 x 18 in, height 18 in, centre of rotation
at the frame centre, intake mouth 16 in wide / 4 in deep / 7 in forward. Measure with a tape and
re-run `npx zenith validate autos/two-tip.auto.json --json`. **Done when** 0 errors with real numbers.

## 7. Waypoints (measure once on a real field)

`autos/waypoints.json` (RED; BLUE is the point mirror). Put the robot where each one says, read the
Pinpoint pose (after setting the start pose), and replace the number and its provenance.

| Waypoint | Current (x, y, heading) | What it is |
|---|---|---|
| `start` | (-13, -63, 90 deg) | The fixed start at the hive that is up at kickoff |
| `scoreSouth` | (-13, -36, 90 deg) | Volleys 1-2, south cell |
| `gardenSweepStart` / `End` | (-44, -61.5) / (-61.5, -61.5), 180 deg | Garden line sweep. **Check the intake reaches the line from y = -61.5** (FIRST's CAD walls are 1.3 in inside the field file's) |
| `gardenBackOut` | (-44, -56, 180 deg) | Room to turn after the garden |
| `flowerWestApproach` / `Pickup` | (-46, -24) / (-57.6, -24), 180 deg | flower0 (near the garden) |
| `scoreNorth` | (-24, 44, -72 deg) | Volleys 3-4, north cell, facing it |
| `flowerNorthPickup` | (-24, 57.6, 90 deg) | flower1 (near the tipped hive) |
| `park` | (-56, 36, 180 deg) | Partly in RED's loading zone |

**Done when** each is measured, the auto re-validates with 0 errors, and `zenith deploy` has been re-run.

## 8. Aim system (teleop)

| Item | Current | Where | How | Done when |
|---|---|---|---|---|
| Hive coordinates | RED (-12.75, 0), BLUE (12.75, 0); cells +/-9.264 in y | `aim/FieldTargets.java` (SPEC, game manual / CAD) | Confirm on the real field | Matches |
| Aim gains | kP 1.2, max yaw 0.7, min yaw 0.06, tolerance 2 deg | `aim/AimController.java` | Hold left trigger from 5 poses | Settles in < 1 s without oscillation |
| Turn rate at full yaw | 5.0 rad/s | `AimController.MAX_TURN_RATE_RAD_PER_S` | Spin at full yaw for 2 s, read the Pinpoint | Measured value; aim holds while strafing |
| Default start pose | (-13, -63, 90 deg) | `teleop/MainHorsTeleOp.java` | Same as the auto's `start` | Matches |

**Check on the real field:** put the robot at a measured pose (e.g. (-40, -40, 0)), hold the left
trigger, and check with a laser or string that the shooter points at the south cell; press A and
check it swings to the north cell.

## 9. SimLoop model numbers (so the sim matches the robot)

`TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/`:

| Item | Current | Where | How |
|---|---|---|---|
| Drive motor / wheel | 312 rpm, 537.7 cpr, 104 mm (assumed) | `BioHorsSim` | Read the real parts |
| Drive response | 5 /s | `BioHorsSim.DRIVE_RESPONSE_PER_S` | Fit a recorded straight-line run (SimLoop `fit` package) |
| Track width / wheelbase | 14 / 12 in | `BioHorsSim` | Tape |
| Shooter spin-up | 3 /s | `BioHorsSim.SHOOTER_RESPONSE_PER_S` | Log rpm from 0 to target; fit the first-order curve |
| Gate transit | full sweep 0.5 s | `BioHorsSim.GATE` | Film the gate |
| Launch cadence, launch cone, flower rate | 0.30 s, 15 deg, 0.15 s | `BiobuzzMatch` | Film volleys and flower pulls |

**Done when** a real run's RLOG and the sim's overlay in AdvantageScope (same pose and rpm curves
within a few percent). SimLoop's `fit` package can compute the gap.

## 10. The tip rule

`autos/field/biobuzz.field.json` `rules.tipTable` is `[8]` (team rule, confirmed by Sushil). The
season package's table (from DSim, the manual has none) is `[8, 7, 6, 3, 1, 0]`, indexed by the
nectar in the up cell, which would let the 3 staged nectar tip the first cell with 3 pollen.
**Confirm on a real field** which is right; if it is the DSim table, the first volley tips the hive
and the auto plan changes.
