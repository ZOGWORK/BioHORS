# This robot (BioHORS) for the build-auto workflow

Read this before step 3 of `SKILL.md`. It is the only place this robot's numbers come from, and
**none of them are measured yet**: every value below is a placeholder or was set from the SimLoop
model. Do not plan a real-field auto around them as if they were real (see `docs/TUNING.md`).

## Intake archetype

- One intake across the FRONT (two motors, `intakeLeft` + `intakeRight`, run together).
  `autos/robot.json` mouth: 13.2 in wide (between the flower mech's side plates), 4.6 in deep, centre
  11.9 in ahead of the drive centre (MEASURED FROM CAD, docs/ROBOT_CAD.md). The body is wider (17.84 in)
  than the mouth, so pollen against a wall are only just reachable.
- Holds **4 pollen** (confirmed by the team; also G407). "Full" = 4.
- **No pollen sensor.** `holdingPiece` / `hopperFull` / `hopperEmpty` are an ESTIMATE
  (`zenith/PollenEstimate`): preload 4, +1 per 0.35 s of inward intake, minus each volley. Always give
  a `wait ... until hopperFull` a `timeoutS`.
- Flowers: nose (intake) to the flower's retrieval opening, centred on it, 0.5 in short of its foot.

## Launcher

- Pollen shooter (`shooterRight`) + pollen gate (`gateRight`); fixed, fires out of the robot's FRONT
  (`AimController.SHOOTER_FACING_OFFSET_RAD = 0`, NEEDS MEASUREMENT). A score pose must FACE the up
  cell (the sim counts a launch within 15 deg of the cell centre) as well as being in its quarter and
  outboard of its outer face.
- `score` count N: spin up (if not already), open the gate, feed with the intake for
  N x 0.4 s + 0.2 s, close the gate. The shooter is left spinning between volleys.

## Speeds (placeholders, kept equal in pedro/Constants and autos/robot.json)

- Top speed 60 in/s forward, 50 in/s strafe; decel 60 / 50 in/s^2 (PLACEHOLDER).
- `accelInPerS2` 220 and Zenith's settle were CALIBRATED FROM SIM (`zenith calibrate --write`).
- Turns on the spot are slower in the sim than Zenith estimates (Pedro's 3 deg heading tolerance).

## Rules this robot's autos follow

- **No mid-path markers.** Zenith v0.1.2's marker runner calls `Follower.curveCompletion()`, which
  Pedro 3.0.2 does not have. Start the intake with a command step before the path instead.
- **Turn on the spot, drive nose-first.** Big heading changes are 1 in "turn" path steps with a
  linear heading; the legs between them are `tangent` or `tangentReversed`. This keeps strafing
  under a fifth of the driving (the match test checks it).
- Do not turn next to a wall: this robot turns about its drive centre, and its far corner (the flower
  bar tip) is 16.8 in from it, so a turn on the spot needs the centre 16.8 in off every wall and
  structure.
- Plan in the field file's frame (walls at +/-72), where its pieces are placed. FIRST's CAD walls are at
  +/-70.67; the field file's garden pollen (x -70.6, radius 1.4) touch its -72 wall, so planning against
  70.67 would put them inside the wall. Which is right is a measure-on-the-field item (docs/TUNING.md 7),
  and any pose within about 1.5 in of a wall is listed there.
- Near a wall, keep the heading square to it: tilting the 22 in body even a few degrees swings the
  bar's corner 14.2 in x the angle toward the wall. The garden lane (x = -62.7) is reached by a short
  sideways slide with the heading held at -90 deg for this reason.
- Tip rule: **8 pollen per tip** (team rule; `autos/field/biobuzz.field.json` rules.tipTable = [8],
  which deliberately differs from the season package's DSim table).

## The headless match (step 12)

```
./gradlew :TeamCode:testDebugUnitTest --tests "*ZenithAutoMatchTest*"
npx zenith sim two-tip --json        # same run through zenith.json's sim command
```

`TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/`: `BioHorsSim` (fake hardware and
physics), `BiobuzzMatch` (pollen, flowers, hive cells, tips, launch legality, contact, strafing),
`TracedZenithAuto` (the deployed auto with Zenith's trace) and `ZenithAutoMatchTest` (RED twice with
identical logs, BLUE once, 30 s stop, AUTO-only score). RLOGs and traces go to `TeamCode/build/sim/`.
