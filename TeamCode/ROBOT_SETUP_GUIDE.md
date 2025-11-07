# Robot Setup Guide - goBILDA 5203 Motors & Logitech Controller

## Hardware Configuration

### Motor Configuration in Driver Station

You'll need to configure these motors in your Robot Controller's configuration file:

#### **Drive Motors (Precision - 312 RPM goBILDA motors)**
- `left_front_drive` - Left front wheel motor
- `right_front_drive` - Right front wheel motor  
- `left_back_drive` - Left back wheel motor
- `right_back_drive` - Right back wheel motor

#### **Mechanism Motors (High Speed - 6000+ RPM goBILDA 5203 series)**
- `arm_motor` - Arm movement (high speed)
- `intake_motor` - Intake/outtake mechanism
- `lift_motor` - Lift mechanism

#### **Servos**
- `claw_servo` - Claw open/close
- `wrist_servo` - Wrist positioning

#### **IMU (Internal Control Hub/Expansion Hub)**
- `imu` - Built-in BNO055 IMU sensor (for field-centric drive)

### IMU Configuration

The Control Hub and Expansion Hub have a **built-in IMU** that works perfectly for field-centric drive:

1. **In Driver Station Robot Configuration:**
   - Add device: `IMU`
   - Device name: `imu`
   - Port: `IMU` (should be available automatically)

2. **IMU Orientation:**
   - Default assumes REV logo facing UP
   - Default assumes USB ports facing FORWARD
   - Adjust in `RobotHardware.java` if mounted differently

### Logitech Controller Mapping

#### **Gamepad 1 (Driver) - Primary Controls**

**Movement:**
- Left Stick Y = Forward/Backward driving
- Left Stick X = Left/Right strafing (mecanum)
- Right Stick X = Rotation (turning)

**Speed Modes:**
- Normal Operation = 80% speed
- Right Bumper = Precision mode (30% speed)
- Left Bumper = Turbo mode (100% speed)

**Mechanisms:**
- A Button = Toggle claw (open/close)
- B Button = Toggle wrist position
- Y Button = Intake forward
- X Button = Intake reverse (outtake)
- DPad Up = Arm up
- DPad Down = Arm down
- Right Trigger = Lift up
- Left Trigger = Lift down

#### **Gamepad 2 (Operator) - Secondary Controls**
- Left Stick Y = Arm control (alternative)
- Right Stick Y = Lift control (alternative)

## Motor Specifications

### Precision Drive Motors (312 RPM)
- **Use Case:** Precise driving, accurate positioning
- **Advantages:** High torque, excellent control, perfect for autonomous
- **Control Tips:** 
  - Can use high power settings (80-100%) safely
  - Excellent for encoder-based positioning
  - Great for precise mecanum drive movements

### High Speed Mechanism Motors (6000+ RPM)
- **Use Case:** Fast mechanisms like arms, intake, lifts
- **Advantages:** High speed operation, quick responses
- **Control Tips:**
  - Use lower power settings (30-60%) for better control
  - Implement smooth ramping for mechanisms
  - Perfect for fast intake/outtake operations

## Created OpModes

### 1. **RobotTeleOpLogitech.java**
- Primary driving OpMode
- Optimized for Logitech controller
- Comprehensive control mapping
- Telemetry feedback

### 2. **AdvancedMecanumDrive.java**
- Advanced driving features
- Acceleration limiting for high RPM motors
- Cubic scaling for smooth control
- Field-relative driving option

### 3. **SimpleAutonomous.java**
- Basic autonomous example
- Shows time-based movement
- Demonstrates mechanism control
- Good starting point for autonomous development

### 4. **MotorTest.java**
- Individual motor testing
- Verify motor directions
- Check motor power levels
- Troubleshooting tool

### 6. **IMUTest.java**
- Test internal Control Hub/Expansion Hub IMU
- Verify IMU is working correctly
- Reset heading functionality
- Visual direction display

### 7. **RobotHardware.java**
- Hardware abstraction class
- Motor and servo initialization
- Helper methods for drive control
- Reusable across all OpModes

## Quick Start Instructions

1. **Configure Robot Controller:**
   - Set up motor and servo names as specified above
   - **Add IMU device named "imu"** for field-centric drive
   - Test each motor individually using MotorTest OpMode

2. **Test IMU Functionality:**
   - Run IMUTest to verify internal IMU is working
   - Check that heading changes when robot rotates

3. **Test Basic Functionality:**
   - Run MotorTest to verify all motors work correctly
   - Check motor directions and adjust in RobotHardware.java if needed

3. **Start Driving:**
   - Use RobotTeleOpLogitech for initial testing
   - Try AdvancedMecanumDrive for more features

4. **Develop Autonomous:**
   - Start with SimpleAutonomous example
   - Expand based on your competition needs

## Motor Power Guidelines

### Drive Motors (312 RPM)
- **Testing:** 60% power
- **Normal Driving:** 80-100% power
- **Precision Mode:** 40% power
- **Autonomous:** 80% power

### Mechanism Motors (6000+ RPM)
- **Arm Movement:** 30-40% power
- **Lift Operation:** 40-60% power  
- **Intake/Outtake:** 60-80% power

## Troubleshooting

### Motor Direction Issues
- If robot moves backward when stick pushed forward, reverse appropriate motors in `RobotHardware.java`
- If robot turns wrong direction, check `rightFrontDrive` and `rightBackDrive` directions

### Precision Drive Motor Control Issues
- 312 RPM motors provide excellent control and can handle full power
- Use encoder feedback for precise autonomous positioning
- Perfect for smooth mecanum drive operation

### High Speed Mechanism Motor Issues
- If mechanisms move too fast, reduce power settings in OpMode
- Use gradual ramping for smooth operation
- Limit power to 30-60% for most mechanism operations

## Next Steps

1. **Test and Tune:** Start with MotorTest, then move to basic driving
2. **Customize Controls:** Modify button mappings to match your team's preferences  
3. **Add Sensors:** Integrate color sensors, distance sensors, IMU for advanced functionality
4. **Develop Autonomous:** Build on SimpleAutonomous example for competition-specific tasks
5. **Add Vision:** Integrate AprilTag or other vision systems for precise autonomous positioning

Happy coding! 🤖