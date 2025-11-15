# TeamCode Module

## Overview

This TeamCode module contains the robot control software for the TurtleWalkers FTC team. The codebase features a sophisticated shooter robot with autonomous capabilities, teleop control, and comprehensive logging for debugging and performance optimization.

## Robot Architecture

### Core Components
- **TurtleRobot.java**: Main robot class with hardware abstraction
- **Shooter Subsystem**: Dual-motor flywheel with turret and hood control
- **Intake Subsystem**: Game piece collection and manipulation
- **Drivetrain Subsystem**: Pedro Pathing integration for precise movement
- **Memory.java**: Persistent state management across OpModes

### Control Systems
- **AutonLinear.java**: Complete autonomous routine with multi-piece scoring
- **Teleop.java**: Traditional gamepad control with manual overrides
- **TeleopNew.java**: Command-based teleop using SolversLib framework

### Vision & Sensors
- **AprilTagLocalization.java**: Vision-based robot localization
- **ArtifactDetector.java**: Game piece detection and tracking

## Key Features

### 🎯 **Precision Shooting System**
- **Dual-motor flywheel** with PID velocity control
- **Turret rotation** with encoder feedback and constraints
- **Variable hood angle** for trajectory adjustment
- **Distance-based auto-aiming** using lookup tables (LUT)
- **Real-time ballistics calculations** for optimal accuracy

### 🚗 **Advanced Drive System**
- **Pedro Pathing integration** for smooth autonomous movement
- **Pose estimation** and field-relative control
- **Alliance-aware positioning** (Red/Blue field adaptation)
- **Precision mode** for fine control during teleop

### 🤖 **Autonomous Capabilities**
- **Multi-phase autonomous** with clear progression tracking
- **Path following** with real-time position updates
- **Dynamic targeting** during movement
- **Error recovery** and emergency handling
- **Competition-ready reliability**

---

# 📊 Extensive Logging System

## Overview

The TeamCode features a comprehensive logging system designed for **adb logcat debugging**. This system provides real-time monitoring, performance analysis, error tracking, and system health monitoring suitable for both competition debugging and development.

## 🎛️ Centralized Configuration

### **LoggingConfig.java** - Single Source of Truth
All logging behavior is controlled through a centralized configuration system:

```java
// Set logging mode for different scenarios
LoggingConfig.setCompetitionMode();   // Minimal logging for matches
LoggingConfig.setDevelopmentMode();   // Maximum logging for debugging  
LoggingConfig.setTestingMode();       // Balanced logging for practice
LoggingConfig.setMinimalMode();       // Only critical errors
```

### **LoggingConfigOpMode** - Interactive Setup
Run the "⚙️ Logging Configuration" OpMode to quickly adjust logging levels:
- **🔧 A Button**: Development Mode (Max Logging)
- **🏆 B Button**: Competition Mode (Min Logging)
- **🧪 X Button**: Testing Mode (Balanced)
- **⚡ Y Button**: Minimal Mode (Errors Only)

## 🔧 Configuration Modes

### **🔧 Development Mode**
- **All logging enabled** for maximum debugging information
- **Best for**: Code development, troubleshooting issues, PID tuning
- **Performance impact**: ~5-10ms per loop
- **Usage**: Full system analysis and problem diagnosis

### **🏆 Competition Mode**
- **Minimal logging** for maximum performance
- **Critical systems only**: Errors, warnings, initialization, autonomous phases
- **Best for**: Actual competition matches
- **Performance impact**: ~1-2ms per loop
- **Usage**: Optimal robot performance with essential monitoring

### **🧪 Testing Mode**
- **Balanced logging** for practice matches
- **Excludes**: Vision processing logs (high overhead)
- **Best for**: Scrimmages, practice matches, driver training
- **Performance impact**: ~2-5ms per loop
- **Usage**: Practice with good debugging capability

### **⚡ Minimal Mode**
- **Errors and warnings only**
- **Best for**: When maximum performance is critical
- **Performance impact**: <1ms per loop
- **Usage**: Emergency performance mode

## 📁 Files Enhanced with Logging

### **Core Robot Systems**
- **TurtleRobot.java**: Hardware initialization, system status, motor health
- **Memory.java**: State changes, consistency checking, alliance tracking

### **Control OpModes**
- **Teleop.java**: Drive inputs, shooter control, turret positioning
- **TeleopNew.java**: Command execution, gamepad monitoring, performance metrics
- **AutonLinear.java**: Phase tracking, path execution, waypoint logging

### **Subsystems**
- **Shooter.java**: PID performance, ballistics calculations, flywheel control
- **Intake.java**: Command tracking, state changes, safety monitoring
- **Drivetrain.java**: Movement tracking, pose updates, path following

### **Vision & Sensors**
- **AprilTagLocalization.java**: Detection events, FPS monitoring, pose updates

## 🔍 ADB Logcat Usage

### **Prerequisites**
1. **Android Debug Bridge (ADB)** installed on development machine
2. **Control Hub** connected via USB with debugging enabled
3. **Driver Station** for wireless adb connection

### **Basic Commands**

```bash
# View all robot logs
adb logcat | grep "TurtleRobot\|AutonLinear\|Teleop"

# Filter by log level
adb logcat | grep "TurtleRobot" | grep "E/"  # Errors only
adb logcat | grep "TurtleRobot" | grep "W/"  # Warnings only
adb logcat | grep "TurtleRobot" | grep "I/"  # Info messages
adb logcat | grep "TurtleRobot" | grep "D/"  # Debug messages

# Save logs to file
adb logcat | grep "TurtleRobot" > robot_debug.log

# Monitor specific subsystems
adb logcat | grep "Shooter\|Intake\|Drivetrain"

# Track performance issues
adb logcat | grep "PERFORMANCE\|Slow loop\|WARNING"
```

### **Competition Workflow**

#### **Pre-Match Setup**
```bash
# 1. Configure logging mode (run LoggingConfigOpMode)
# 2. Verify minimal overhead
adb logcat | grep "LoggingConfig.*Competition"

# 3. Monitor critical systems only  
adb logcat | grep "ERROR\|WARNING\|AutonLinear"
```

#### **Match Analysis**
```bash
# Quick match log capture
adb logcat | grep "AutonLinear\|Performance" > match_analysis.log

# Monitor autonomous phases
adb logcat | grep "PHASE.*===.*AutonLinear"

# Check for errors during match
adb logcat | grep "ERROR\|EMERGENCY" --color=always
```

#### **Post-Competition Debugging**
```bash
# Switch to development mode for detailed analysis
# Run LoggingConfigOpMode and select Development Mode

# Full system debugging
adb logcat | grep "TurtleRobot\|Shooter\|PID" > detailed_debug.log

# Performance analysis
adb logcat | grep "Performance.*Avg loop time" 
```

## 🎯 Logging Features

### **Initialization Tracking**
- Hardware component setup with timing
- Error detection with stack traces
- Voltage monitoring and warnings
- PID controller configuration
- Lookup table construction

### **Real-time Performance Monitoring**
- Loop timing analysis (target <20ms)
- Battery voltage tracking with alerts
- Motor current consumption monitoring
- PID controller performance metrics
- Path following efficiency tracking

### **System Health Monitoring**
- Motor overcurrent detection (>10A threshold)
- Servo position validation
- Sensor reading verification
- Communication error detection
- Hardware component status

### **Competition-Specific Logging**
- Autonomous phase identification with clear progression
- Alliance-specific behavior tracking (Red/Blue)
- Emergency shutdown logging with context
- State consistency validation
- Real-time performance alerts

### **Debug & Development Features**
- **PID Tuning**: Detailed controller performance data
- **Ballistics Analysis**: Distance-RPM-angle calculations
- **Vision Processing**: AprilTag detection and FPS monitoring
- **Command Tracking**: Button presses and system responses
- **Error Recovery**: Exception handling with graceful degradation

## 📋 Log Message Format

All logs follow a consistent format for easy parsing:

```
[TIMESTAMP] [LEVEL]/[TAG]: [MESSAGE]
```

### **Example Log Entries**
```
11-14 15:30:25.123 I/TurtleRobot: === TurtleRobot initialization started ===
11-14 15:30:25.145 I/AutonLinear: === PHASE 1: Preload Shot ===
11-14 15:30:25.167 W/TurtleRobot: WARNING: Battery voltage is low! Current: 11.2V
11-14 15:30:25.189 D/Shooter: Shooter PID - Target: 400.0 RPM, Current: 395.2 RPM, Error: 4.8
11-14 15:30:25.201 E/TurtleRobot: ERROR in Shooter: Motor overcurrent detected
```

## 🔧 Usage Examples

### **In OpMode Initialization**
```java
// The logging system is automatic, but you can add custom logs:
TurtleRobot robot = new TurtleRobot(this);
robot.init(hardwareMap);
robot.logSystemStatus(); // Optional: Force status check
```

### **During Operation Loop**
```java
while (opModeIsActive()) {
    // Automatic logging happens based on LoggingConfig settings
    
    // Add custom logging for specific events
    if (gamepad1.a) {
        RobotLog.ii("CustomOpMode", "Shooter activated by driver");
    }
    
    // Performance monitoring is automatic
    // System health checks are automatic  
    // PID logging is automatic
}
```

### **Error Handling**
```java
try {
    // Robot operations
    robot.shooterb.setPower(power);
} catch (Exception e) {
    // Automatic error logging with stack traces
    robot.logError("Shooter", "Failed to set power: " + e.getMessage());
}
```

## 🚀 Best Practices

### **Competition Usage**
1. **Run LoggingConfigOpMode** before each match to set Competition Mode
2. **Monitor critical logs** only: `adb logcat | grep "ERROR\|WARNING"`
3. **Save match logs** for post-game analysis
4. **Switch to Development Mode** only for debugging between matches

### **Development & Debugging**
1. **Use Development Mode** for maximum visibility during testing
2. **Monitor PID performance**: `adb logcat | grep "PID.*Target.*Current"`
3. **Track initialization issues**: `adb logcat | grep "initialization"`
4. **Analyze performance**: `adb logcat | grep "Performance.*loop time"`

### **Performance Optimization**
1. **Monitor loop times** for performance warnings (>25ms)
2. **Check battery voltage** for low power alerts (<11.5V)
3. **Watch motor current** for overcurrent warnings (>10A)
4. **Use Testing Mode** for practice matches to balance debugging and performance

## 🎯 Benefits

### **For Development**
- **Faster debugging**: Real-time issue identification
- **Performance optimization**: Loop timing and efficiency tracking
- **Hardware validation**: Comprehensive component health monitoring
- **PID tuning**: Detailed controller performance data with error analysis

### **For Competition**
- **Match analysis**: Complete autonomous and teleop execution tracking
- **Failure diagnosis**: Emergency conditions with recovery context
- **Performance monitoring**: Real-time alerts without impacting robot performance
- **Post-match debugging**: Detailed logs for issue reproduction and analysis

### **For Team Management**
- **Historical analysis**: Long-term performance trends and hardware wear
- **Configuration validation**: Setup verification and consistency checking
- **Training aid**: Driver performance and robot behavior correlation
- **Competition readiness**: Confidence through comprehensive monitoring

---

## 🗂️ Project Structure

```
TeamCode/
├── autonomous/          # Autonomous OpModes
│   ├── AutonLinear.java      # Main autonomous routine
│   ├── AutonRed.java         # Red alliance specific
│   ├── AutonBlue.java        # Blue alliance specific
│   └── DumbAuto.java         # Simple autonomous backup
├── teleop/             # Teleoperated OpModes
│   ├── Teleop.java           # Traditional gamepad control
│   └── TeleopNew.java        # Command-based control
├── subsystems/         # Robot subsystem classes
│   ├── Shooter.java          # Flywheel and turret control
│   ├── Intake.java           # Game piece manipulation
│   └── Drivetrain.java       # Movement and positioning
├── robot/              # Core robot classes
│   ├── TurtleRobot.java      # Main robot hardware class
│   └── Memory.java           # Persistent state management
├── camera/             # Vision processing
│   ├── AprilTagLocalization.java  # Pose estimation
│   └── ArtifactDetector.java      # Game piece detection
├── pedroPathing/       # Path following configuration
│   ├── Constants.java        # Pathing parameters
│   └── Tuning.java          # Path following tuning
├── shooter/            # Legacy shooter classes
└── util/               # Utility classes
    ├── LoggingConfig.java         # Centralized logging control
    └── LoggingConfigOpMode.java   # Interactive logging setup
```

## 🚀 Getting Started

### **For New Team Members**
1. **Study TurtleRobot.java** to understand hardware layout
2. **Review AutonLinear.java** for autonomous strategy
3. **Practice with Teleop.java** for driver control
4. **Use LoggingConfigOpMode** to configure debugging level

### **For Competition**
1. **Set Competition Mode** for optimal performance
2. **Test autonomous** with minimal logging overhead
3. **Monitor critical systems** during matches
4. **Save logs** for post-competition analysis

### **For Development**
1. **Enable Development Mode** for full debugging
2. **Use adb logcat** for real-time monitoring
3. **Tune PID controllers** with detailed performance logs
4. **Validate hardware** with comprehensive health monitoring

The TeamCode is competition-ready with extensive debugging capabilities, providing the tools needed for both high-performance competition play and rapid development iteration.

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

1) Locate the desired sample class in the Project/Android tree.

2) Right click on the sample class and select "Copy"

3) Expand the  TeamCode/java folder

4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

5) You will be prompted for a class name for the copy.
   Choose something meaningful based on the purpose of this class.
   Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.

Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
    package="org.firstinspires.ftc.teamcode"
    to be
    package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.

6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""