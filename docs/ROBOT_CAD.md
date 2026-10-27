# Robot geometry from CAD

Source: `33950 Robot.step` (Onshape export, Oct 27, 61 MB), read with OpenCascade (XCAF) part by
part. Every number below is labelled **MEASURED FROM CAD** in the files that use it. CAD can be out
of date with the real robot, so each one still wants a tape-measure check (`docs/TUNING.md` section 6).

## Frames

The STEP's own frame has the intake at -x and the shooters at +x, the nectar side at -y, and the
floor at z = -1.105 in. The robot frame used everywhere else (Pedro, Zenith, the sim) is:

| Robot frame | From CAD |
|---|---|
| forward | 1.735 - cadX (the drive centre is at cadX = 1.735) |
| left | -cadY |
| up | cadZ + 1.105 |

## Numbers

| What | Value | Used in |
|---|---|---|
| Overall box | 22.0 x 17.84 x 14.9 in (long x wide x high) | `autos/robot.json` footprint, height |
| Flower bar tip | 14.2 in ahead of the drive centre | footprint, `centreOfRotationIn` (-3.2 in) |
| Frame front / back | 9.6 in ahead / 7.8 in behind | start pose: centre 7.8 in off the wall |
| Drive wheels | 104 mm, at forward +5.2 / -5.2 in, left +/-6.48 in | sim: wheel base 10.39, track width 12.96 |
| Pinpoint | forward -5.2, left -0.32 in | (reference) |
| X (forward) pod wheel | 2.84 in to the right of the drive centre | `pedro/Constants` `xPodOffset` -2.84 |
| Y (strafe) pod | **not in the CAD** | NEEDS MEASUREMENT |
| Intake mouth | 13.2 in wide (between the flower mech side plates), 4.6 in deep, centre 11.9 in ahead | `autos/robot.json` intake mouth |
| Flower mech bar | 0.8 to 1.5 in above the floor, built to slide into a flower's retrieval opening | flower pickup poses |
| Shooters | nectar on the left, pollen on the right, one motor each, 1:1 | `MechanismConstants` |
| Flywheels | 72 mm Rhino wheel (radius 1.42 in) plus a 60 mm steel flywheel, axle about 10.5 in up | shot model (next PR) |
| Shot exit | forward over the intake, about 12 in up; pollen ramp about 44 deg, nectar hood about 49 deg | shot model (next PR) |
| Gate servo | nectar side | (reference) |

## Open questions for the team

1. **Start size.** With the flower bar out the robot is 22 in long, over an 18 in start box. Does the
   flower mech fold in for the start? If it does, `startIn` becomes the 17.44 in frame.
2. **Y pod.** The strafe odometry pod is not in the CAD; its forward offset (`yPodOffset`) is still a
   placeholder.
3. **Flower bar into the foot.** The bar is built to enter the retrieval opening, but the field file
   treats the foot as solid, so the auto stops the bar 0.5 in short. Find the seated pose on a real field.

## Re-reading the STEP

The reader is a short OCP (cadquery-ocp) script: load with `STEPCAFControl_Reader` (names on), walk
the XCAF label tree applying each instance's location, and write each part's name path and world
bounding box. Loading the 61 MB file takes about 90 s; gmsh's STEP import hangs on it.
