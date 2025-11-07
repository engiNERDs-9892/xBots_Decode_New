# Main TeleOp Controller - Control Reference

## 🎮 **Gamepad 1 (Driver) Controls**

### **Movement (Left Stick)**
- **Up/Down**: Forward/Backward drive
- **Left/Right**: Strafe left/right (mecanum)

### **Rotation (Right Stick)**
- **Left/Right**: Robot rotation/turning

### **Speed Modes**
- **Normal**: 80% speed (default)
- **Right Bumper**: Precision mode (40% speed)
- **Left Bumper**: Turbo mode (100% speed)

### **Drive Modes**
- **START Button**: Toggle field-centric/robot-centric drive
- **BACK Button**: Reset IMU heading to 0°

### **Mechanisms**
- **DPad Up**: Arm up
- **DPad Down**: Arm down
- **DPad Left**: Intake forward (moved from Y button)
- **DPad Right**: Intake reverse/outtake (moved from X button)
- **Right Trigger**: Lift up (hold) / Claw toggle (when pressed fully > 80%)
- **Left Trigger**: Lift down
- **A Button**: **Launcher Motor** - Hold for 4500 RPM (5203 series 6000rpm)
- **Y Button**: **Pickup Motor** - Hold for 100 RPM (5203 series 6000rpm)
- **X Button**: **Kicker Motor** - Hold for 150 RPM (5203 series 312rpm)
- **B Button**: Toggle wrist position

## 🎮 **Gamepad 2 (Operator) Controls**

### **Alternative Mechanism Control**
- **Left Stick Y**: Arm control (up/down)
- **Right Stick Y**: Lift control (up/down)
- **Right Bumper**: Intake forward
- **Left Bumper**: Intake reverse

## 📊 **Telemetry Display**
- Current speed mode and drive mode
- Robot heading (when IMU available)
- Motor power levels
- Mechanism status
- Servo positions
- **PID Motor**: Target vs Current RPM, active status, control instructions

## 🎯 **PID Motor Control Systems**

### **Launcher Motor (A Button)**
- **Target Speed**: 4500 RPM when A button is held
- **Motor Type**: 5203 series 6000rpm goBILDA motor
- **Purpose**: High-speed launching mechanism
- **Tuning**: kP=0.01, kI=0.001, kD=0.0001

### **Pickup Motor (Y Button)**
- **Target Speed**: 100 RPM when Y button is held
- **Motor Type**: 5203 series 6000rpm goBILDA motor
- **Purpose**: Slow, controlled pickup operations
- **Tuning**: kP=0.02, kI=0.005, kD=0.0001 (higher gains for low speed)

### **Kicker Motor (X Button)**
- **Target Speed**: 150 RPM when X button is held
- **Motor Type**: 5203 series 312rpm goBILDA motor
- **Purpose**: Precise kicking and positioning
- **Tuning**: kP=0.05, kI=0.01, kD=0.0005 (high gains for precision)

### **Common Features**
- **Control Method**: Advanced PID algorithm for precise speed control
- **Encoder**: Uses built-in encoders for accurate RPM measurement
- **Safety**: Automatic shutoff when buttons are released
- **Live Feedback**: Real-time RPM display in telemetry

## 🔧 **Hardware Requirements**
- Drive motors configured as: `left_front_drive`, `right_front_drive`, `left_back_drive`, `right_back_drive`
- Mechanism motors: `arm_motor`, `intake_motor`, `lift_motor`
- **PID Motors**: 
  - `launcher_motor` (5203 series 6000rpm for 4500 RPM launcher)
  - `pickup_motor` (5203 series 6000rpm for 100 RPM pickup)
  - `kicker_motor` (5203 series 312rpm for 150 RPM kicker)
- Servos: `claw_servo`, `wrist_servo`
- IMU: `imu` (optional, for field-centric drive)

## 🚀 **Features Ready for Expansion**

### **Section Markers in Code:**
1. **ADDITIONAL FEATURE VARIABLES** - Add new state variables
2. **ADDITIONAL FEATURES SECTION** - Add new control logic
3. **ADDITIONAL METHODS SECTION** - Add helper methods
4. **Telemetry section** - Add status displays for new features

### **Easy Addition Points:**
- Vision processing controls
- Automated sequences
- Competition-specific functions
- Advanced sensor integration
- Custom control algorithms

## 💡 **Usage Tips**
1. **Start in robot-centric mode** for initial testing
2. **Use precision mode** for careful positioning
3. **Enable field-centric** after IMU is configured
4. **Reset heading** when robot faces away from driver station
5. **Use gamepad 2** for operator-assisted control during competition