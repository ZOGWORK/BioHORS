# Drive the teleop in the sim with a controller

Plug a USB game controller into the laptop, run one command, and drive `MainHorsTeleOp` (unchanged)
on the simulated robot and the BIOBUZZ field, in real time. A window shows the field from above
with the Driver Station telemetry beside it. Every session is recorded as an RLOG you can open in
AdvantageScope afterwards, plus the controller input that made it, so you can replay it exactly.

No robot, no Control Hub, no Driver Station.

## Run it

1. Plug in the controller **before** starting. Xbox (wired or Bluetooth), Logitech F310 (switch on
   the back set to **X**), PS4 or PS5 pads all work. A second controller, if plugged in, is
   gamepad2.
2. From the project root, in PowerShell:

   ```powershell
   .\gradlew.bat driveSim
   ```

   (macOS/Linux: `./gradlew driveSim`.) Gradle needs Java; if it says `JAVA_HOME` is not set, use
   the one Android Studio ships with:

   ```powershell
   $env:JAVA_HOME = 'C:\Program Files\Android\Android Studio\jbr'
   ```

3. The window opens: 3 s of INIT (the Pinpoint calibrates), then PLAY. The robot starts at the
   auto's start spot (RED by default) with the auto's 4 pollen aboard, so you can shoot at once.
   Drive with the normal teleop controls (README, "Teleop controls"): left trigger aims, Y volleys,
   right trigger intakes, and so on.
4. The session lasts the teleop period, 2:00. To stop sooner, **hold BACK + START for 1 s** (on a
   PlayStation pad: Share + Options), or close the window. The rest of the period then runs in a
   second or two with the sticks at rest, so the log is still complete.

The console then prints where the files went:

```
[live] RLOG (drag into AdvantageScope): ...\TeamCode\build\sim\live\drive-20261104-213000.rlog
[live] input track (replay with -Preplay=<it>): ...\TeamCode\build\sim\live\drive-20261104-213000.gamepad.txt
```

## Options

Add any of these to the command, for example `.\gradlew.bat driveSim -Palliance=BLUE -Pseconds=60`.

| Option | Default | What it does |
|---|---|---|
| `-Palliance=BLUE` | `RED` | Which alliance; the start spot is mirrored for BLUE |
| `-Pseconds=60` | `120` | Length of the driver period, in seconds (after the 3 s INIT) |
| `-Ppreload=0` | `4` | Pollen aboard at the start |
| `-Pwindow=false` | `true` | No window (drive blind; for when there is no display) |
| `-Preplay=latest` | | Replay the newest session's input instead of reading a controller (below) |
| `-Pwatch=true` | `false` | With `-Preplay`: play it back in real time in the window |

PowerShell splits an argument at a dot, so quote any option whose value has one:
`"-Preplay=TeamCode/build/sim/live/drive-20261104-213000.gamepad.txt"`, `"-Pseconds=90.5"`.

## Watch it in AdvantageScope

Drag the session's `.rlog` into [AdvantageScope](https://docs.advantagescope.org/), open a
**3D Field** tab and pick **2026-2027 Field**, then from `RealOutputs/Field` drag `Robot` onto the
field and set `Pollen` to **Game Piece → Pollen** (as in the README, "Watch a run on the 3D
field"). `Match/held`, `Match/scored`, `Match/tips` and `Shooter/pollenRpm` plot on a Line Graph.

AdvantageScope shows the session after it ends, not while you drive: the pinned SimLoop
(`v0.2.0-alpha.2`) writes an OpMode run's log only to the file (`OpModeRunner.runLogged` takes no
extra log receivers, and PsiKit ignores receivers added once logging has started), so there is no
live stream to connect to. The window is the live view.

## Replay a session

```powershell
.\gradlew.bat driveSim -Preplay=latest
```

replays the newest session's input track through the same teleop and sim, as fast as it can, into
`<session>-replay.rlog`, and checks it logs exactly what the session logged. Add `-Pwatch=true` to
watch it in the window in real time. Replays are how you show a bug to someone: send the
`.gamepad.txt`, and the same code gives the same run. After a code change, a replay shows what the
new code does with the same driving.

## How it works

`driveSim` runs one JUnit class, `sim/LiveDriveSession`, and nothing else (it is a session, not a
test: it only runs when asked for). It builds the robot sim (`BioHorsSim`) and the field
(`BiobuzzMatch`, physics mode), and runs the teleop with SimLoop's `OpModeRunner`. Each 20 ms
loop, in order:

1. `GlfwGamepadSource` polls the controller (GLFW, through LWJGL, test-only) and SimLoop's
   `GamepadState.applyTo` puts it on gamepad1 / gamepad2, as a Driver Station packet would;
   `RecordingGamepadInputSource` writes the same frame to the input track.
2. The teleop's loop runs; the fake devices, plants and field update; the window is told.
3. SimLoop's `RealTimePacer` waits until the wall clock catches up with sim time. It only waits,
   so a replay without it computes the same run.

Sim time comes from SimLoop's `FakeTimer`; the pacer is the only thing that reads the wall clock
(and the file names). The window is `simdesktop/FieldWindow`, a small plain-Java module, because
TeamCode's test code compiles against Android's library, which has no window toolkit. None of it
(LWJGL, the window, SimLoop) reaches the APK.

`LiveDriveTest` checks the plumbing without a controller in every normal test run: scripted input
reaches the teleop and moves the robot, a recorded track replays to an identical RLOG, watching
does not change a run, BACK + START ends it, and GLFW's layout maps onto the FTC gamepad.

## Limits

- **Every sim number is a placeholder** (`docs/TUNING.md`): the robot's speed, flywheel spin-up and
  the field model are not measured. Driving here trains the controls and checks the code's logic,
  not the real robot's handling.
- The field is `BiobuzzMatch`, built for the AUTO match: its class comment says what it models
  (pollen as physics balls, the intake mouth, launches, hive tips) and what it does not.
- No rumble: the teleop's rumble cues go to the simulated gamepad, not the controller.
- A controller must be plugged in when the session starts; unplugging one mid-session reads as
  sticks at rest.
- The window draws the robot as an 18 in square and the hives as bars; AdvantageScope has the real
  models.
- If the laptop cannot keep up with real time, the console says how often the sim fell behind.
