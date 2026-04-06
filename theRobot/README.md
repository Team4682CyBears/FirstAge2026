# FirstAge 2026 Robot Code
## Team 4682 CyBears — Bishop Blanchet High School

This repository contains the 2026 FRC robot code for **Team 4682 CyBears**, built using WPILib 2026.1.1 and the WPILib Command Framework.

> **Context:** This robot is a "rebuilt" concept based on the 2024 Crescendo shooter architecture, applied to the 2026 FirstAge game. It combines the shooting and aiming systems refined in 2024 with the swerve and vision improvements developed in 2025. For the original Crescendo implementation, see the [Crescendo2024 repository](../../Crescendo2024).

---

## Project Overview

The robot is a **holonomic swerve drive system** with an integrated **shooting mechanism**, **intake system**, **climber**, and **vision-based aiming capabilities**. The codebase follows the WPILib Command Framework pattern with subsystem-based architecture.

### Subsystem Code Leads (2026)
| Subsystem | Lead(s) |
|-----------|---------|
| Autos | Asher |
| Auto Aim (shooter, turret, azimuth) | Isaac |
| Swerve + Odometry | Asher + Mateo |
| Climb + climber deploy | Sachin |
| Intake | Jasper |
| Indexer / Feeder | Leo (w/ Jasper assist) |
| Flex / LEDs | Jasper + Mateo |

### Key Features:
- **Swerve Drivetrain**: Field-centric and robot-centric driving modes with auto-aiming capability
- **Vision System**: Limelight-based AprilTag detection with MT2 pose estimation
- **Shooter System**: Dual SparkFlex motors with RPM control and hood angle adjustment
- **Turret**: Independent targeting mechanism with limit switch homing
- **Intake**: Deployable wrist with roller motor
- **Spindexer**: Ball indexing system with optional ToF sensor feedback
- **Kicker**: Separate mechanism to feed balls into shooter
- **Climber**: Height-controlled mechanism with Hall Effect sensor (currently disabled)
- **LED System**: Status indication (currently disabled)

---

## Hardware Configuration

### Motor Controllers & Motors
- **Drivetrain**: TalonFX motors (CTRE) with CANcoder encoders
- **Shooter**: 2x SparkFlex motors (REV) - lead/follow configuration
- **Hood**: TalonFXS motor with CANcoder absolute encoder
- **Turret**: TalonFX motor with digital limit switches
- **Intake Wrist**: TalonFX with CANcoder absolute encoder
- **Intake Roller**: TalonFX motor
- **Kicker**: TalonFX motor
- **Spindexer**: TalonFX motor with optional ToF sensor
- **Climber**: 2x SparkMax motors with Hall Effect sensor (DISABLED)
- **LED**: PWM output (DISABLED)

### Sensors
- **Vision**: Limelight 4 (AprilTag detection)
- **IMU**: Pigeon 2.0 (in drivetrain)
- **Encoders**: CANcoder (hood, turret, intake wrist), internal encoder feedback
- **ToF Sensors**: Optional spindexer sensor for ball detection
- **Hall Effect**: Climber limit detection
- **Digital Input**: Turret limit switches (2x)

### CAN Device IDs
| Device | CAN ID(s) | Notes |
|--------|-----------|-------|
| Drivetrain modules | 1–12 | TalonFX + CANcoder pairs (4 modules × 3 each) |
| Pigeon 2.0 IMU | 13 | On CANivore |
| Intake roller motor | 16 | Kraken/TalonFX |
| Intake wrist motor | 17 | Kraken/TalonFX |
| Spindexer motor | 18 | Kraken/TalonFX |
| Kicker motor | 19 | Kraken/TalonFX |
| Shooter follow motor | 20 | Rev Vortex/SparkFlex |
| Shooter lead motor | 21 | Rev Vortex/SparkFlex |
| Turret motor | 23 | TalonFX |
| Climber lead motor | 25 | Kraken (disabled) |
| Climber follow motor | 26 | Kraken (disabled) |
| Spindexer LaserCAN sensor | 27 | Grapple Robotics LaserCAN |
| Full hopper LaserCAN sensor | 28 | Grapple Robotics LaserCAN |
| PDH | 29 | Rev Power Distribution Hub |
| Hood motor | 30 | CTRE Minion/TalonFXS |
| Hood thru-bore encoder | 31 | CANcoder |
| Intake wrist thru-bore encoder | 32 | CANcoder |

### DIO Ports
| Port | Device |
|------|--------|
| DIO 0 | Climber Hall Effect sensor |
| DIO 1 | Turret limit switch (primary) |
| DIO 2 | Turret limit switch (secondary) |

### PWM Ports
| Port | Device |
|------|--------|
| PWM 0 | LED strip |

### Power Distribution
The robot uses a REV Power Distribution Hub (PDH, CAN 29) with the following layout:

**VRM Power Budget** (3 VRMs on one 15A PDH port via Wago 221 splice):
| Component | Qty | Draw |
|-----------|-----|------|
| CANcoders | 5 | 0.30 A |
| Pigeon 2.0 | 1 | 0.04 A |
| LaserCAN / TOF sensors | 2 | 0.20 A |
| Magnetic sensor | 1 | 0.01 A |
| CANivore | 1 | 0.10 A |
| LED strip (~64 LEDs) | 1 | ~4.00 A |
| **Total** | | **~4.65 A** |

---

## Core Subsystems

### 1. DrivetrainSubsystem
Controls the swerve drive with support for:
- **Field-centric driving**: Control based on field orientation
- **Robot-centric driving**: Control based on robot heading
- **Auto-aiming mode**: Automatic yaw rotation to face targets
- **Vision fusion**: Odometry updates from Limelight AprilTags
- **PathPlanner integration**: Autonomous path following

Key Constants:
- Max velocity: 5.0 m/s
- Max angular velocity: 657 deg/s
- Vision standard deviation: (0.7, 0.7, 100) for position, rotation
- Odometry std dev: (0.1, 0.1, 0.01)

### 2. ShooterAimer
Complex helper class that calculates optimal shooting parameters:
- **Distance-to-target calculation**: From robot pose to hub or shuttle
- **Velocity prediction**: Accounts for robot and target motion
- **Turret offset compensation**: Calculates separate turret angle vs. robot yaw
- **Lookup tables** for distance-based control:
  - Hood extension vs. distance
  - Shooter RPM vs. distance
  - Kicker RPM vs. distance
  - Time-of-flight vs. distance
- **ProfiledPID**: Smooth yaw velocity generation

Lookup Table Distances: 1.0m to 8.27m

### 3. ShooterSubsystem
Dual SparkFlex motors with closed-loop velocity control:
- Runs at commanded RPM (0-6500 RPM)
- P=0.00025, I=0, D=0.001
- Feedforward: kS=0.065, kV=0.00172
- Idle mode: Coast
- Current limit: 40A smart current

### 4. HoodSubsystem
TalonFXS with external CANcoder feedback:
- Extension range: 0 to 0.635 rotations
- MotionMagic control (velocity 800 rot/s, accel 160 rot/s²)
- PID: kP=0.4, kI=0.02, kD=0, kV=0.58, kS=0.10, kG=0.024
- Tolerance: 0.01 rotations

### 5. TurretSubsystem
Independent turret aiming:
- Angle range: 0° to 355° relative to robot
- Motor with limit switch homing (attempts both sensors)
- ProfiledPID control (kP=0.14, kI=0, kD=0)
- Tolerances: 0.018 radians position, low velocity check
- Safety features: Voltage limit checks, gear ratio compensation

Limit Switch Positions:
- Primary (DIO 1): 7° offset
- Secondary (DIO 2): -8° offset

### 6. IntakeWristSubsystem
Deployable intake arm with MotionMagic control:
- Positions:
  - Deployed: 0 rotations
  - Defensive/Stowed: 0.511 rotations
  - Agitate position: 0.3439 rotations
- Gear ratio: (1/5 * 1/5) rotor-to-sensor, (18/32) sensor-to-mechanism
- MotionMagic: velocity 50 rot/s, accel 25 rot/s²
- PID: kP=0.2, kI=0.003, kD=0, kV=2.5, kS=0.44

### 7. IntakeRollerSubsystem
Simple velocity-controlled intake:
- Runs at commanded RPM (typically 5000 RPM)
- PID: kP=0.4, kI=0, kD=0
- Feedforward: kS=0.090, kV=0.450
- Idle mode: Coast

### 8. KickerSubsystem
Kicks balls into shooter:
- Target RPM: 2000
- PID: kP=0.52, kI=0, kD=0
- Feedforward: kS=0.120, kV=0.109
- Gear ratio: 3:1

### 9. SpindexerSpinner
Feeds balls from intake to kicker:
- Continuous mode: Always run at 150 RPM
- Sensor mode: Stop when ToF detects ball (optional)
- PID: kP=0.4, kI=0, kD=0
- Feedforward: kS=0.120, kV=0.109
- Gear ratio: 25:1

### 10. ClimberSubsystem (DISABLED)
Position and velocity control for climbing:
- Control via encoder position in inches (custom conversion)
- Hall Effect sensor for zeroing at top
- Min height: 20.75 inches, Max: 28 inches
- Position control: kP=0.2, kI=0, kD=0
- Feedforward: kS=0.15, kV=0.002

### 11. CameraSubsystem
Limelight 4 integration:
- **MT1 (MegaTag 1)**: Single-tag pose estimation
- **MT2 (MegaTag 2)**: Multi-tag robust estimation (primary)
- **Seeding mode**: Uses MT1 yaw history for rotation, MT2 for position
- **Tracking mode**: Standard MT2 with IMU assist
- Fiducial ambiguity threshold: 0.2
- Auto-switches camera IMU based on mode

---

## Command System

### Drive Commands
- **DefaultDriveCommand**: Teleop field-centric or auto-yaw driving from joysticks
  - Left stick Y: Forward/backward
  - Left stick X: Strafe
  - Right stick X: Rotation (joystick mode) or auto-rotation (auto mode)

### Shooter Commands
- **AutoAimCommand**: Continuous aiming using ShooterAimer
  - Calculates hood angle and shooter RPM based on distance
  - Reduces drive power during aiming
  - Handles turret aiming mode switching

- **ShooterManualCommand**: Fixed-position shooting
  - Hood: 0.12 rotations (close range)
  - Shooter: 3000 RPM
  - Turret: 90° (fixed)

- **ShootCommand**: Simple shooter velocity command

### Intake Commands
- **ToggleIntakeDeployCommand**: Deploy/retract intake wrist
  - Deployed: Run roller at dashboard-set RPM
  - Retracted: Stop roller

- **IntakeWristManualCommand**: Manual wrist control via joystick
- **IntakeRollerManualCommand**: Manual roller control

### Feeding Commands
- **KickerSpindexerAgitateCommand**: Feed balls with intake motion
  - Runs kicker + spindexer continuously
  - Toggles intake wrist between deployed/agitate position every 0.5s
  - Used during shooting to feed multiple balls

- **KickerSpindexerCommand**: Feed balls without agitation
- **SpindexerCommand**: Spindexer-only control

### Turret Commands
- **TurretDefaultCommand**: Auto-aim using ShooterAimer target angle
- **TurretTestPositionCommand**: Move to specific angle for testing
- **ToggleTurretAimModeCommand**: Switch between auto/manual modes

### Climber Commands (DISABLED)
- **ClimberPositionCommand**: Move to target height
- **ClimberVelocityCommand**: Manual velocity control via joystick

### Utility Commands
- **AllStopCommand**: Emergency stop all subsystems
- **DriveTimeCommand**: Drive for specified duration
- **ButtonPressCommand**: Log button presses
- **FollowTrajectoryCommandBuilder**: Build PathPlanner trajectory commands

---

## Controller Bindings

### Driver (Xbox 0)
| Button | Action |
|--------|--------|
| Back | Zero gyroscope |
| Start | Toggle camera seeding mode |
| Left Trigger | Reduce drive power (pit mode) |
| Right Trigger | Kicker+Spindexer+Agitate (hold) |
| X | EMERGENCY STOP |
| Y | Auto-aim (hold) |
| B | Toggle intake deploy/retract |
| Right Bumper | Manual shoot mode (hold) |
| POV Up/Right/Down/Left | Turret test positions (90°/180°/270°/0°) |

### Co-Driver (Xbox 1)
| Button | Action |
|--------|--------|
| X | EMERGENCY STOP |
| Y | Toggle intake roller (hold) |
| Left Bumper | Climber to min height (disabled) |
| Right Bumper | Climber to 28" (disabled) |
| Start | Toggle turret aim mode |
| Left Joystick Y (>0.1) + B | Manual intake wrist control |
| Right Joystick Y (>0.1) + B | Manual climber velocity (disabled) |

---

## Autonomous Routines

Auto modes are managed via `AutonomousChooser` and use PathPlanner:

1. **Do Nothing**: Instant command (safe default)
2. **Just Shoot**: Aim + shoot (5s timeout)
3. **Bot Wing Poo**: PathPlanner routine
4. **Top Wing Poo**: PathPlanner routine
5. **Hub Outpost Depo**: PathPlanner routine

Auto features:
- Named commands: AutoAimOn, SpindexerKickerOn, IntakeToggle, RevShooter
- Mirror paths based on alliance color
- Holonomic path follower with PathPlanner
- PID: Translation (kP=2.0), Rotation (kP=4.5, kI=0.001)

---

## Vision System

**Limelight 4** provides AprilTag-based localization:

- **Standard MT1**: Single tag detection, good for fallback
- **MegaTag 2 (MT2)**: Multi-tag robust estimation (primary)
- **Seeding Mode**: Combines MT1 yaw history (median of 15 samples) with MT2 position
- **Camera IMU**: Set based on mode (seeding mode 1, tracking mode 4 with IMU assist alpha=0.01)
- **Pose Estimation**: Dual-matrix Kalman filtering with std dev:
  - Vision: [0.7, 0.7, 100] (high rotation uncertainty)
  - Odometry: [0.1, 0.1, 0.01]

Shooter offset from robot center:
- X: -0.1397m (aft)
- Y: -0.2032m (starboard)
- Yaw: -60° relative to robot heading

### Auto Aim Architecture
`ShooterAimer` coordinates across drivetrain, shooter, and hood subsystems each tick:

**Outputs computed:**
- `targetRobotYaw` — desired robot heading for azimuth alignment
- `targetHoodExtension` — hood angle from distance lookup table
- `targetShooterVelocity` — shooter RPM from distance lookup table
- `isShotFeasible` — whether conditions allow a shot
- `isAtPosition` — whether robot yaw, hood, and shooter are all on target

**Targets supported:**
- Stationary auto-aim at blue/red hub
- Shoot-on-the-fly with robot velocity compensation
- Shuttle targets (mid-field, auto-selects nearest side)
- Operator `TargetAdjustment` via co-driver D-pad offset

**Drivetrain in shoot mode:** restricts max robot speed and maintains auto yaw. Turret handles fine azimuth; swerve yaw handles coarse alignment.

---

## Hardware Status Flags

Set in `InstalledHardware.java`:

```
powerDistributionPanelInstalled: true
limelightInstalled: true
drivetrainInstalled: true
shooterInstalled: true
hoodMotorInstalled: true, hoodEncoderInstalled: true
turretInstalled: true, turretSensorInstalled: true, turretSecondSensorInstalled: true
spindexerInstalled: true, spindexerSensorInstalled: false
kickerInstalled: true
intakeWristInstalled: true, intakeWristEncoderInstalled: true, intakeRollerInstalled: true
climberInstalled: false (DISABLED)
LEDSInstalled: false (DISABLED)
useTurretForAiming: true (vs. swerve yaw)
```

---

## Telemetry & Logging

The robot logs data via:
- **DataLogManager**: High-frequency CSV logs to RoboRIO
- **AdvantageScope**: Recommended for post-match analysis and debugging (see `SensorData/` folder for example datasets)
- **Elastic Dashboard**: Recommended for driver station display in 2026 — Shuffleboard and SmartDashboard are **deprecated** in WPILib 2026. See [Elastic docs](https://frc-elastic.gitbook.io/docs).
- **NetworkTables**: Published swerve module states
- **Optional ShotLogger**: Track shooting events (currently disabled)

> **Note on dashboards:** The team is mid-transition from Shuffleboard to Elastic for driver display and AdvantageScope for debugging. Some SmartDashboard calls remain in the code from prior years and will be migrated.

Key telemetry outputs:
- Shooter RPM, Hood position, Turret angle
- Intake wrist position, mode
- Climber position/velocity (if enabled)
- Vision measurements, camera state
- Drive power, swerve state
- Button presses logged as CommandScheduler events

---

## Code Organization & Patterns

### Design Patterns
1. **Subsystem-based**: Each mechanism is a `SubsystemBase`
2. **Command-driven**: All actions are `Command` implementations
3. **Registry pattern**: `SubsystemCollection` centralizes subsystem access
4. **Config flags**: `InstalledHardware` allows rapid hardware changes
5. **Lookup tables**: Interpolating tables for shooting parameters

### Key Classes
- **SubsystemCollection**: Central registry for all subsystems
- **ManualInputInterfaces**: Translates controller input to commands
- **ShooterAimer**: Complex ballistic calculator (physics-based)
- **Constants**: All tuning values in one place
- **LookupTableDouble**: 1D interpolating lookup tables with clamping

---

## Testing & Debugging

### Test Files
Located in `src/test/java/frc/robot/`:
- `LookupTableDoubleTest.java`: Lookup table interpolation
- `ShootOnTheFlyTest.java`: Auto-aiming calculations
- `ShootStationaryTest.java`: Stationary shooting
- `TurretTest.java`: Turret homing and limits

Run tests:
```bash
./gradlew test
```

### Debug Features
- Diagnostic paths enabled in Constants (put test trajectories on dashboard)
- Turret limit switch states published to SmartDashboard
- Shooter/Hood/Intake encoder positions logged
- Vision diagnostics: Fiducial count, ambiguity, heartbeat
- ButtonPressCommand logs all button events

---

## Development Notes

### Dependencies (from build.gradle)
- WPILib 2026.1.1 with all FRC libraries
- Rev Robotics (SparkFlex, SparkMax)
- CTRE (TalonFX, TalonFXS, CANcoder, Pigeon)
- PathPlanner 2026.1.0 (or later) for autonomous
- Third-party vendor libraries (auto-downloaded)

### Sensor Selection: LaserCAN vs. CANrange vs. PWF
The team evaluated three TOF sensors for ball detection in 2026. **LaserCAN (Grapple Robotics) was selected** as the primary sensor. Key findings from empirical testing:

| Sensor | Min detectable | Accuracy (1–7") | Notes |
|--------|---------------|-----------------|-------|
| LaserCAN | ~0.5" | ±0.25" | Best accuracy; requires USB-B cable to set CAN ID |
| CANrange (CTRE) | ~1.14" | ±0.4" | Good CAN integration; less accurate than LaserCAN |
| PWF (Playing with Fusion) | ~1.6" | Up to ±2" | Least accurate; not recommended for close-range detection |

See `Code/Coding26/Sensor Documentation.docx` for full testing methodology, distance tables, and dynamic detection (spinning ball) results.

### Future Improvements
- Enable climber subsystem and integrate into auto
- Tune ShooterAimer lookup tables post-competition
- Complete AdvantageScope simulation support (WPILib simulation requires per-subsystem additions — partially started by Asher)
- Use Limelight object detection for game piece pickup assistance (Sachin's 2026 project)
- Post-game AdvantageScope dashboards for subsystem health monitoring

---

## Related Repositories & Resources

| Resource | Notes |
|----------|-------|
| [Crescendo2024](../../Crescendo2024) | Original shooter/vision architecture this robot is based on |
| [Reefscape2025](../../Reefscape2025) | Elevator/TOF/Phoenix 6 patterns; swerve foundation used here |
| [ChargedUp2023](../../ChargedUp2023) | First swerve implementation for context |

### Team
**Team 4682 CyBears — Bishop Blanchet High School, Seattle WA**
**Season**: 2026 (FirstAge)

---

*README generated from source code analysis + Rebuilt SW Spec (Coding26).*
