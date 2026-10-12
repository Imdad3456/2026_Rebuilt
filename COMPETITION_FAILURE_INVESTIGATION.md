# Competition Failure Investigation: "Auto works, then the whole robot stops in teleop"

**Status: root cause NOT confirmed.** This document ranks the explanations by evidence, lists
the confirmed code defects, and describes the diagnostics added so the next occurrence can be
pinned down. Nothing here should be read as "the original failure is fixed". The code builds and
the tests pass, but the failure has never been reproduced and no competition logs were available.

> **Code structure has changed since this report.** The refactor that followed it removed the
> `MotorControllerWrapper` / `RealSparkMax` / `MockSparkMax` / `HardwareFactory` layer (now
> `util/hardware/Sparks.java`) and the single-purpose command classes (`LauncherOn`,
> `HomeExtender`, …), which are now command methods on each subsystem. File names below
> describe the code **as it was at competition**.
>
> **Field-relative "offset kept changing":** this was bug #10 below. The navX's `getAngle()` is
> clockwise-positive, but WPILib's field-relative math expects counter-clockwise. Every degree
> the robot turned moved the field-relative heading two degrees the wrong way, so "forward" kept
> drifting as the robot rotated. It now uses `getRotation2d()`. "Forward" is a software offset
> zeroed at the start of auto or with the driver's zero-heading button, not `navX.reset()`, which
> applies asynchronously. If the gyro disconnects, driving falls back to robot-relative.

## The symptom

- The robot worked normally in the pits.
- On the field, autonomous ran.
- When teleop started, nothing responded: drivetrain, intake, shooter, both controllers.
- It happened repeatedly.

The most important clue is **"everything, from both controllers"**. A bug in one subsystem or
command can't do that. Something robot-wide has to fail:

| Robot-wide cause | What it does |
|---|---|
| Driver Station / FMS communication loss (including a radio reboot) | The roboRIO disables all outputs |
| Brownout (battery below ~6.75 V) | The roboRIO disables all outputs, CAN motor controllers stop |
| Joysticks not delivering input | Robot is enabled and healthy, but receives no commands |
| Robot code crash / restart | No robot code for 10+ seconds, then it comes back |
| Main loop stall | All commands and subsystems stop updating |

The investigation checked the code for each of these.

## Which code actually ran at competition?

**Confirmed: the competition-era commit `397c1e4` (Feb 24) does not compile.** `RobotContainer`
uses `CanIdConstants.kLauncherMotor`, which commit `0aecbea` renamed. So the robot at competition
was running either:

- `2932205` (Feb 14), which compiles, but uses the **old CAN ID map**, or
- `397c1e4` plus uncommitted local edits that were never pushed.

`2932205` and `397c1e4` differ **only** in `CanIdConstants.java`. All the logic analyzed below is
identical in both. The CAN ID maps are very different, though. For example, the swerve modules are
assigned to different corners, and the intake moves from 13 to 10 and the launcher from 15 to 35.
Running code whose IDs don't match the wiring would make whole mechanisms dead. That would
probably show up in the pits too, so it is unlikely to be *the* cause, but it can't be ruled out.

**How to find out exactly what ran:** `build.gradle` copies `src/` into the deployed jar
(`backup/src`). **Before deploying new code**, copy the jar off the robot:

```
scp lvuser@roboRIO-5919-FRC.local:/home/lvuser/*.jar .
```

Then open it as a zip and read `backup/src/main/java/frc/robot/constants/CanIdConstants.java`.
If the robot has been redeployed since the event, that evidence is gone.

From now on every build stamps its git commit, branch, uncommitted-change flag and build time
(`BuildInfo`). It is shown on the dashboard as `Build` and written to the log at startup.

## Ranked explanations

### 1. Robot-wide outage: Driver Station communication loss or brownout (most likely, unconfirmed)

- **Why it fits:** It is the only class of cause that stops *everything* from *both* controllers.
  It is load-dependent, which explains pits vs field: on blocks the wheels spin freely and draw
  little current, while on carpet, at full stick, the drivetrain draws far more. Autonomous
  ("drive straight at half speed for 3 s") is gentle. Teleop is the first time the robot is
  driven hard. If the radio reboots after a voltage dip, the robot is gone for 30 to 60+ seconds,
  which is most of teleop.
- **Code factors that existed at competition:**
  - Drive motors limited at 60 A each (`DriveConfigs`, `GlobalConstants.kHighCurrentLimit`).
  - Steering motors created with the *drive* config, also 60 A (`MAXSwerveModule.java`). See #2.
    On closer analysis their effective gain was so small that they probably drew little current,
    so their contribution to brownout is uncertain.
  - No logging of battery voltage, brownouts or comms, so it was never visible.
- **Confirmed or hypothetical:** Hypothetical. Battery, wiring and radio power are just as
  likely as code.
- **Already fixed:** Drive limit reduced to 50 A and steering config corrected (`8e5921f`).
  Diagnostics added (below). This does **not** fix an electrical fault.
- **What settles it:** The competition Driver Station logs, opened in the FRC Driver Station
  Log Viewer on the laptop that was used at the event. Look for the voltage trace dropping toward
  6.8 V, brownout markers, or "comms lost" right after teleop starts.

### 2. Drivetrain could not steer under load (confirmed defect, existed at competition)

- **File:** `src/main/java/frc/robot/subsystems/MAXSwerveModule.java` (constructor), introduced
  in `3c2957a` "mock motor controllers" on Feb 1.
- **Code at competition:**
  ```java
  m_turningMotor = HardwareFactory.createSparkMax(turningCANId, DriveConfigs.MAXSwerveModule.drivingConfig);
  ```
  Combined with `RealSparkMax.m_useAbsolute` never being set, every steering motor:
  - closed its loop on the motor's built-in relative encoder instead of the absolute encoder,
    zeroed wherever the wheel happened to point at power-on;
  - used the drive conversion factor, so 1 commanded "radian" was about 172 degrees of wheel;
  - used the drive P gain (0.04), giving only about 2 to 4% output for a large angle error;
  - ran in coast mode with a 60 A limit.
- **Why it fits:** Auto only drives straight (angle 0), so the wheels never need to steer, and
  that works. In teleop, any strafe or rotation asks the wheels to steer. On blocks the unloaded
  wheels can turn on a few percent output. On carpet, under the robot's weight, they can't, so
  the drivetrain looks unresponsive or erratic. That matches "pits fine, field broken" **for the
  drivetrain**.
- **What it does not explain:** the intake, shooter and operator controls also failing.
- **Confirmed or hypothetical:** Defect confirmed by code and git history. Its role in the
  incident is hypothetical.
- **Already fixed:** Yes, in `8e5921f` (turning config + absolute encoder). **Not yet verified on
  the robot.** Use the swerve checks in the checklist below.

### 3. Controllers not reaching the robot on the field (hypothetical)

- **File:** `RobotContainer.java`. The driver is fixed to port 0 and the operator to port 1,
  with no check that either is connected or is the expected device.
- **Why it fits:** A field Driver Station that enumerated the controllers in a different order,
  or lost one when USB was re-plugged, would give "both controllers do nothing" while the robot
  itself is healthy. Auto doesn't use the controllers, so auto would still look fine.
- **Confirmed or hypothetical:** Hypothetical. No evidence either way.
- **Already fixed:** Not a code bug. The new diagnostics log, per port, whether a controller is
  connected, its name, type, axis count and button count, and every connect/disconnect. The DS
  data log records every axis and button value.

### 4. Robot code crash or main-loop stall (no evidence found)

The full auto → disabled → teleop path was traced in `Robot.java`, `RobotContainer.java`,
`DriveCommand.java` and every subsystem `periodic()`:

- `teleopInit()` cancels the auto command correctly. `DriveStraight` requires the drivetrain,
  and the default `DriveCommand` resumes when it ends. This is now covered by tests.
- No code path throws in teleop. Missing joystick axes or buttons produce warnings, not exceptions.
- REV getters read cached CAN frames and don't block.
- The only blocking calls in periodic code are motor `configure()` calls in the "retune while
  disabled" blocks, and the extender's 50 Hz `println` when skewed (see bugs table).
- **Confirmed or hypothetical:** No evidence of a crash or stall in the code. The real `Robot`
  was run in simulation through three auto → disabled → teleop cycles without error
  (`RobotSmokeTest`), but with simulated motors.
- **Diagnostics added:** exceptions are logged before the program restarts. Loop period and
  `robotPeriodic` time are logged. A program-start event says whether the roboRIO itself rebooted
  (power loss) or only the code restarted.

### 5. Motor configuration rewritten in the gap between auto and teleop (hypothetical, low)

- **Files:** every subsystem `periodic()` with `if (DriverStation.isDisabled()) { if (X.hasChanged()) setPID(...) }`
  (`DriveSubsystem`, `ExtenderSubsystem`, `LauncherSubsystem`, `FeederSubsystem`,
  `IntakeSubsystem`, `Rollers`). These were added in `9686e6b` "tuning shuffleboard" on Feb 14,
  right before competition.
- **Why it could matter:** On the field the robot is disabled for a few seconds between auto and
  teleop. If a dashboard value changed then, for example from a dashboard re-sending its values
  after reconnecting, each subsystem would run blocking `configure()` calls over CAN, and the
  drivetrain does it for all 8 swerve controllers. That is a short stall, not a whole dead teleop.
- **Already fixed:** `TunableNumber.hasChanged()` now always returns false while the FMS is
  attached (and when `TuningManager.kTuningMode` is false). Changes are applied after the match.
  This is covered by `TunableNumberTest`.

### 6. CAN bus failure (possible, never established)

Nothing in the code floods the bus: REV getters read periodic frames, and setpoints are normal
periodic control frames. A bus-off condition is almost always wiring, termination, or a damaged
device. The diagnostics now record bus utilization, bus-off, TX-full and receive/transmit error
counters, plus each SPARK's CAN fault and "has reset" flags.

## Confirmed code bugs

| # | File | Bug | At comp? | Status |
|---|---|---|---|---|
| 1 | `RobotContainer.java` | `CanIdConstants.kLauncherMotor` doesn't exist, so `397c1e4` doesn't compile | Yes, means comp code ≠ `397c1e4` | Fixed `8e5921f` |
| 2 | `MAXSwerveModule.java` | Steering motors created with the driving config | Yes (since Feb 1) | Fixed `8e5921f`, verify on robot |
| 3 | `RealSparkMax.java` | `m_useAbsolute` never set, absolute encoder never used | Yes | Fixed `8e5921f` |
| 4 | `RealSparkMax.updateHardwarePID` | Forced the feedback sensor to the primary encoder on every retune | Yes | Fixed `8e5921f` |
| 5 | `RealSparkMax.follow` | Follower told to follow *itself* | Yes | Fixed `8e5921f` |
| 6 | `ExtenderSubsystem.periodic` | Commanded both extender motors independently, and the follower had no gains | Yes | Fixed `8e5921f` (hardware follower) |
| 7 | `ExtenderSubsystem.emergencyStop` | `System.out.println` every 20 ms while skewed, flooding the console stream to the DS | Yes | Fixed `8e5921f` (latched, reported once) |
| 8 | `LauncherOn.java` | Speed slider read once, at robot boot | Yes | Fixed `8e5921f` |
| 9 | `DriveConfigs.java` | 60 A drive limit (REV recommends 50 A for MAXSwerve) | Yes | Fixed `8e5921f` |
| 10 | `DriveSubsystem.java` | navX `getAngle()` (clockwise positive) used as a CCW-positive heading, so heading and odometry were mirrored | Yes (field-relative was off, so driving was unaffected) | Fixed in this change |
| 11 | All tunable `periodic()` blocks | Motor configs could be rewritten while on the FMS | Yes | Fixed in this change |
| 12 | `RealSparkMax.java` | `configure()` results ignored, so a failed config (missing or duplicate CAN ID, timeout) was silent | Yes | Fixed in this change (logged) |

### Needs a measurement on the robot, not changed: swerve chassis offsets

`MAXSwerveModule.getState()` and `getPosition()` subtract `m_chassisAngularOffset`, but
`setDesiredState()` does not add it. That is the same as the pre-refactor code the team drove
with. Which half is right depends on how the absolute encoders were zeroed, and that can only be
measured:

1. Robot disabled and on blocks. Turn all four wheels by hand to point straight forward, with the
   bevel gears all facing the same side.
2. Read `/Diag/Drive/<Module>/MeasuredAngleRad` in AdvantageScope (live NetworkTables or the log).
   - **All about 0:** encoders are zeroed to the robot frame. Set all four offsets in
     `DriveConstants` to `0`. Driving is unchanged, and odometry/`getChassisSpeeds()` become correct.
   - **About the REV template values** (FL ≈ 4.71, FR ≈ 0, RL ≈ 3.14, RR ≈ 1.57): the encoders use
     REV's convention. `setDesiredState()` must add the offset
     (`desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))`), or the wheels
     will point the wrong way.

No offset change was made without this measurement. Guessing wrong would turn modules 90 degrees.

## Fixes in this change

- **`TunableNumber.hasChanged()`**: no motor configuration writes while on the FMS or with
  tuning mode off.
- **`DriveSubsystem`**: heading from `m_gyro.getRotation2d()` (CCW positive), and `getTurnRate()`
  sign to match.
- **`RealSparkMax`** (now `Sparks`): every `configure()` result is checked, and failures are logged once with
  the CAN ID. All real SPARK MAX controllers are registered for fault monitoring.
- **`RobotContainer`**: auto choices are named ("Do Nothing", "Simple Auto") so the logs say
  which ran. The drive command is kept for diagnostics.
- **`build.gradle`**: generates `BuildInfo` (git commit, branch, uncommitted-changes flag, build time).

## Diagnostic system

Everything is written by WPILib's `DataLogManager` to a `.wpilog` on the roboRIO
(`/home/lvuser/logs`, or a USB stick if one is plugged in). It is **not** sent over
NetworkTables. Values use `update()`, so they are written only when they change. Slow signals are
sampled at 10 Hz or 2 Hz. SPARK faults are read one controller per loop from frames the SPARKs
already broadcast, so no extra CAN traffic is created. Messages go to the Driver Station console
only for rare, important transitions.

| Area | Log entries | Rate |
|---|---|---|
| Timeline | `/Diag/Events`: program start (build, rio-reboot vs code-restart), every mode change, AUTO/TELEOP/DISABLED init, auto-cancel result, every command start/end/interrupt, DS connect/disconnect, comms-disable, brownout start/end, joystick connect/disconnect, SPARK faults/resets, CAN bus-off, slow loops, exceptions | on event |
| Robot state | `/Diag/State/Enabled`, `Autonomous`, `Teleop`, `FMSAttached`, `DSAttached`, `OutputsActive`, `EStopped`, `CommsDisableCount`, `MatchTime` | 50 Hz, on change |
| Power | `/Diag/Power/BatteryVoltage`, `BrownedOut` | 10 Hz |
| Controllers | `/Diag/Joystick0/…`, `/Diag/Joystick1/…` (`Connected`, `Description`). Raw axes and buttons: `DS:joystick*` from `DriverStation.startDataLog` | 10 Hz / DS rate |
| Drive input path | `/Diag/Drive/DriveCommandScheduled`, `InputX`, `InputY`, `InputRot` (values after deadband, as sent to the drivetrain) | 50 Hz, on change |
| Swerve | `/Diag/Drive/<Module>/CommandedAngleRad`, `MeasuredAngleRad`, `CommandedSpeedMps`, `MeasuredSpeedMps`, `/Diag/Drive/GyroConnected` | 10 Hz |
| CAN bus | `/Diag/CAN/Utilization`, `BusOffCount`, `TxFullCount`, `ReceiveErrorCount`, `TransmitErrorCount` | 2 Hz |
| Motors | `/Diag/Motors/CANxx/FaultBits`, `WarningBits`, `LastError` | each SPARK about 3 Hz |
| Performance | `/Diag/Loop/PeriodSeconds`, `RobotPeriodicSeconds`, `OverrunCount`. The WPILib watchdog also prints per-subsystem times on overruns. | 50 Hz |

**Reading it:** In AdvantageScope, use *File → Download Logs* to pull the log from the robot,
then open it. Start with `/Diag/Events` as a table, then plot `BatteryVoltage`, `OutputsActive`,
`DSAttached` and `DriveCommandScheduled` on one timeline. Also keep the Driver Station's own logs
(`C:\Users\Public\Documents\FRC\Log Files` on the DS laptop) for every match.

### Telling the causes apart

| Cause | What the logs show |
|---|---|
| DS communication loss | `DSAttached` false and/or `CommsDisableCount` increases, `OutputsActive` false, event `DS DISCONNECTED` / `COMMS DISABLE`. A radio reboot also shows as a long gap on the DS log viewer |
| Joystick disconnected / wrong port | Robot enabled, `OutputsActive` true, but `/Diag/Joystick0/Connected` false or `Description` is the wrong device, and `InputX/Y/Rot` stay 0 |
| Code crash | `EXCEPTION` event, then a new `ROBOT PROGRAM START` saying "robot CODE restarted". The DS shows "No Robot Code" |
| Scheduler/loop stall | `RobotPeriodicSeconds` large, `SLOW LOOP` events, `OverrunCount` rising, with DS still attached |
| Brownout | `BatteryVoltage` near or below 6.75 V, `BrownedOut` true, `BROWNOUT START` event, `OutputsActive` false. SPARK `RESET` events afterwards. A `ROBOT PROGRAM START` saying "roboRIO BOOTED" means the roboRIO lost power |
| CAN bus fault | `BusOffCount`/error counters rising, `CAN BUS-OFF` event, SPARK `CAN-FAULT` or `LastError` = `kTimeout`/`kCANDisconnected`, while DS and power look healthy |
| Motor config problem | `[MOTOR] CAN xx … FAILED` at boot, or the swerve `Commanded` and `Measured` angles disagreeing while everything else is healthy |

## Competition reproduction checklist

**Safety first:** have an adult mentor present, keep a hand on the enable/disable controls
(Enter disables, Space e-stops), clear the area, and run first on blocks, then on carpet with a
tethered or practice-field setup and room to move.

1. **Before deploying:** copy the old jar off the robot (see "Which code actually ran"), then deploy.
2. Check the dashboard `Build` string shows the commit you expect, with no uncommitted changes.
3. Fully charged battery. Note its voltage at rest.
4. Power on the robot. Connect the **competition** driver and operator controllers to the
   **competition** DS laptop, in the competition order. On the DS USB tab confirm driver = 0 and
   operator = 1, and lock the order.
5. Connect wirelessly through the radio, not tethered. The tether hides radio and bandwidth problems.
6. Select the autonomous you ran at competition.
7. In the Driver Station choose **Practice** mode. It runs auto → disabled → teleop with real
   timings, which matches the field transition more closely than enabling teleop by hand.
8. During teleop, on carpet: drive hard in all directions, strafe, spin in place. Run every
   operator function (intake in/out, extender out/in, shooter, idle on/off).
9. Confirm both controllers and every mechanism respond for the whole teleop period.
10. Repeat steps 7 to 9 at least five times, including back-to-back runs on the same battery.
11. After each run, download the `.wpilog` and the DS log. Check `/Diag/Events` for any of the
    signatures in the table above, even if the run looked fine.
12. **Swerve checks** (first run only, on blocks): do the chassis-offset measurement above, and
    confirm that `CommandedAngleRad` and `MeasuredAngleRad` match within a few degrees while steering.
13. **Extender check** (on blocks): move it out and in. If `Extender/Status` reports a skew fault
    immediately, the follower's encoder direction is opposite to the leader's. Report it. Don't
    raise the skew limit.

**What can't be reproduced here:** the real FMS, the field radio and its bandwidth limit, and
field-carpet load. Practice mode with the real radio and carpet is the closest equivalent. If the
failure only ever appears on a real field, the new logs from that match are what will identify it.

## Tests

| Test | What it checks | Needs the robot? |
|---|---|---|
| `ModeTransitionTest` | Auto takes the drivetrain, `DriveCommand` resumes after cancel, the FMS sequence auto → disabled → teleop, repeated matches | No |
| `RobotSmokeTest` | The real `Robot` (simulated hardware) through 3 auto → disabled → teleop cycles; `DriveCommand` running in teleop | No (motors are simulated mocks, so REV faults aren't exercised) |
| `TunableNumberTest` | Tuning changes ignored on the FMS, applied after | No |
| `RobotUtilsTest` | Existing math helpers | No |

Run with `./gradlew build` (all pass). These **cannot** test comms loss, brownouts, CAN faults,
joystick enumeration or swerve hardware. Those need the checklist above on the real robot.

## Remaining uncertainties

1. **The root cause is unconfirmed.** The competition DS logs (on the laptop used at the event)
   are the single most valuable piece of evidence. The only log on this laptop is a few seconds long.
2. **Which code ran at competition.** Retrievable from the jar on the roboRIO if it hasn't been
   redeployed.
3. **Swerve offsets.** Needs the measurement above.
4. **Electrical health** (battery age, radio power, CAN wiring and termination, main breaker,
   power connectors). Software can't detect or fix these, but the logs will now show their effects.
5. **The second launcher motor (CAN 36)** is not configured. Its required direction is unknown.
6. **The extender follower's encoder direction** in hardware-follow mode needs the on-robot check.
