# TeamCode OpModes Documentation

This document describes the function of each OpMode program that runs the robot.

## Main OpModes

### TeleOpMain
**Group:** Linear OpMode  
**File:** `OpModes/Main/TeleOpMain.java`

The primary teleoperated control mode for competition. Integrates all major robot subsystems:
- **DriveTrain**: Mecanum drive control via left stick (forward/right) and right stick (rotate)
- **Turret**: Automatic alignment to AprilTag 24 using Limelight vision
- **Launcher**: Flywheel control for shooting
- **Spindexer**: Ball indexing and shooting sequence with color detection
- **Controls**: 
  - Gamepad 1: Drive train, spindexer controls (A/B/X/Y), shooting
  - Automatically starts flywheel when X is pressed
  - Stops flywheel after 3 shots

### AutoHoodAndShoot
**Group:** Main  
**File:** `OpModes/Main/AutoHoodAndShoot.java`

Automatically adjusts hood angle and flywheel power based on distance to AprilTag target:
- Uses Limelight vision to calculate distance via Target Area (TA) method
- **Left Trigger**: Activates hood adjustment based on distance
- **Right Trigger**: Activates flywheel power adjustment based on distance
- **Square Button**: Activates both hood and shooter simultaneously
- Automatically calculates optimal hood position and flywheel power for given distance

### TurretAutoAlign
**Group:** Main  
**File:** `OpModes/Main/TurretAutoAlign.java`

PID-controlled turret alignment system:
- Uses Limelight vision to detect AprilTag 24
- PID controller automatically aligns turret to target
- Locks position when within tolerance range
- Handles tag loss with smart scanning behavior
- Provides telemetry for tuning PID constants

### ManualHoodAndShoot
**Group:** Individual Test  
**File:** `OpModes/Main/ManualHoodAndShoot.java`

Manual control mode for testing hood and shooting systems:
- **Left Trigger**: Increase flywheel RPM (rate-limited)
- **Left Bumper**: Decrease flywheel RPM (rate-limited)
- **Right Trigger**: Increase hood angle (rate-limited)
- **Right Bumper**: Decrease hood angle (rate-limited)
- **Circle**: Toggle flywheel on/off
- **Cross (X)**: Trigger kicker sequence
- Includes odometry tracking via IMU with odometry pods
- Displays prediction model data (odometry, limelight distance, hood position, flywheel speed)

### MainSpindexer
**Group:** Linear OpMode  
**File:** `OpModes/Main/MainSpindexer.java`

Standalone spindexer control mode:
- Color detection for purple and green balls
- **Button A**: Move spindexer one division (60°)
- **Button B**: Reset spindexer to initial position
- **Button X**: Shoot ball sequence (finds correct color, moves to position, kicks)
- **Button Y**: Toggle intake on/off
- Tracks ball colors in 3 divisions
- Shooting sequence includes kicker servo control

### FlyWheelDistanceShooter
**Group:** Main  
**File:** `OpModes/Main/FlyWheelDistanceShooter.java`

Flywheel power control based on distance:
- Uses Limelight vision to calculate distance to AprilTag
- Implements rolling average filter for stable distance readings
- Automatically adjusts flywheel power based on calculated distance
- **Circle**: Toggle flywheel on/off
- Provides telemetry for distance and power values

### DriveTrain
**Group:** Main  
**File:** `OpModes/Main/DriveTrain.java`

Basic drive train control mode for testing mecanum drive:
- Standard mecanum wheel control
- Left stick: Forward/backward and strafe
- Right stick: Rotation

### HoodCameraControl
**Group:** Main  
**File:** `OpModes/Main/HoodCameraControl.java`

Hood servo control with camera feedback:
- Manual hood position adjustment
- Camera-based feedback for positioning

## Individual Test OpModes

### BallLauncherServo
**Group:** ProgrammingBoardShooter  
**File:** `OpModes/IndividualTest/BallLauncherServo.java`

Individual test for ball launcher servo:
- Tests Axon Micro CR servo
- Button-based direction control

### ColorSensor
**Group:** Individual Test  
**File:** `OpModes/IndividualTest/ColorSensor.java`

Color sensor testing and calibration:
- Tests color detection for purple and green balls
- Displays raw sensor values
- Useful for tuning color detection thresholds

### FlyWheel
**Group:** Individual Test  
**File:** `OpModes/IndividualTest/FlyWheel.java`

Individual flywheel motor test:
- Direct control of flywheel motors
- Power adjustment for testing
- Useful for tuning flywheel speeds

### HoodServoControl
**Group:** Individual Test  
**File:** `OpModes/IndividualTest/HoodServoControl.java`

Individual hood servo control:
- Manual position control
- Step-by-step position adjustment
- Useful for finding optimal hood positions

### Intake
**Group:** Test  
**File:** `OpModes/IndividualTest/Intake.java`

Intake system testing:
- Tests intake servo/motor control
- Useful for tuning intake speeds

### kicker
**Group:** Individual Test  
**File:** `OpModes/IndividualTest/kicker.java`

Kicker servo control:
- Individual test for kicker servo
- Position control for testing kicker sequence

### Limelight
**Group:** Main  
**File:** `OpModes/IndividualTest/Limelight.java`

Limelight vision system testing:
- Displays raw Limelight data
- Tests AprilTag detection
- Useful for vision system calibration

### Parking
**Group:** Individual Control  
**File:** `OpModes/IndividualTest/Parking.java`

Parking mechanism testing:
- Tests parking servo/motor
- Individual control for testing

### PieSectionTest
**Group:** Test  
**File:** `OpModes/IndividualTest/PieSectionTest.java`

Spindexer pie section testing:
- Tests individual pie sections
- Useful for calibrating spindexer positions

### PieThong
**Group:** Test  
**File:** `OpModes/IndividualTest/PieThong.java`

Full servo hardware test:
- Comprehensive test for all servo hardware
- Useful for hardware verification

### SpindexerTest
**Group:** Linear OpMode  
**File:** `OpModes/IndividualTest/SpindexerTest.java`

Spindexer positional servo test:
- Tests 720° positional servo
- Manual position control
- Useful for calibrating spindexer movements

## In Progress OpModes

### IndexingOrShootPie
**Group:** Linear OpMode  
**File:** `OpModes/InProgress/IndexingOrShootPie.java`

**Status:** In Development

Early version of spindexer system:
- CRServo-based indexing
- Color detection integration
- Being replaced by MainSpindexer

### IndexingOrShootPie2
**Group:** Linear OpMode  
**File:** `OpModes/InProgress/IndexingOrShootPie2.java`

**Status:** In Development

Second iteration of spindexer system:
- Improved version of IndexingOrShootPie
- Being replaced by MainSpindexer

### IndexingOrShootSystem
**Group:** Linear OpMode  
**File:** `OpModes/InProgress/IndexingOrShootSystem.java`

**Status:** In Development

Integrated indexing and shooting system:
- Combines spindexer and shooter
- Work in progress

### colorsensortest
**Group:** Tests  
**File:** `OpModes/InProgress/colorsensortest.java`

**Status:** In Development

Color sensor testing with dark filter:
- Purple/Green detector V3
- Dark filter only
- Testing and calibration mode

## Pathing OpModes

### Tuning
**Group:** Pedro Pathing  
**File:** `OpModes/pedroPathing/Tuning.java`

Path following system tuning:
- Tunes PIDF constants for path following
- Various test patterns (straight line, curve, triangle, circle)
- Forward/backward velocity tuning
- Zero power acceleration tuning
- Translational and heading PIDF tuning

## Component Classes

The robot uses a component-based architecture located in `OpModes/Main/Components/`:

- **DriveTrain**: Mecanum drive train control
- **Flywheel**: Flywheel motor control
- **Launcher**: Combined flywheel and hood control
- **Spindexer**: Ball indexing and color detection system
- **Turret**: Turret servo control with Limelight vision alignment

## Constants

Configuration constants are stored in `Constants/`:
- **Drive.java**: Drive train constants (robot speed)
- **Shooter.java**: Shooter constants (camera height, target height, mount angle)
- **Intake.java**: Intake constants (currently empty)

## Notes

- All OpModes follow FTC SDK conventions
- TeleOp modes use `@TeleOp` annotation
- Linear OpModes extend `LinearOpMode` for sequential operations
- Regular OpModes extend `OpMode` for iterative operations
- Vision-based OpModes require Limelight hardware
- Some OpModes require specific hardware configurations (check hardware map names)
