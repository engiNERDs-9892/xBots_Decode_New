# PID Motor Setup Guide

## 🔧 **Hardware Configuration Required**

To use the PID motor control feature in the Main TeleOp Controller, you need to add a motor named `pid_motor` to your robot configuration.

### **Steps to Configure:**

1. **Connect Your G203 Series Motor**
   - Connect your first G203 series 6000rpm goBILDA motor to any available motor port on your Control Hub or Expansion Hub
   - Note which port you used (e.g., Motor Port 0, 1, 2, or 3)

2. **Robot Configuration (Driver Station)**
   - Open the FTC Driver Station app
   - Tap **Configure** 
   - Select your robot configuration or create a new one
   - Navigate to the motor port where you connected the G203 motor
   - Set the motor name to: **`pid_motor`** (exactly as shown)
   - Save the configuration

3. **Motor Direction (if needed)**
   - The PID system will work regardless of motor direction
   - If you need to reverse the motor direction, you can modify the RobotHardware.java file
   - Add this line in the motor configuration section: `pidMotor.setDirection(DcMotor.Direction.REVERSE);`

## ⚙️ **PID Tuning (Advanced)**

The default PID values are optimized for goBILDA motors:
- **kP = 0.01** (Proportional gain)
- **kI = 0.001** (Integral gain) 
- **kD = 0.0001** (Derivative gain)

### **If you need to adjust PID values:**

1. **Too much oscillation?** → Reduce kP and kD
2. **Too slow to reach target?** → Increase kP
3. **Can't maintain steady speed?** → Increase kI (but keep it small)
4. **Overshooting target?** → Reduce kP, increase kD

### **To modify PID values:**
Edit the variables in MainTeleOpController.java around line 74:
```java
private double kP = 0.01;     // Proportional gain
private double kI = 0.001;    // Integral gain  
private double kD = 0.0001;   // Derivative gain
```

## 🎯 **Usage Instructions**

1. **Deploy** the updated code to your robot
2. **Run** the "Main TeleOp Controller" OpMode
3. **Hold the A button** on gamepad 1 to activate PID control
4. **Watch telemetry** to see target vs actual RPM
5. **Release A button** to stop the motor

## 📊 **Telemetry Information**

When PID control is active, you'll see:
- **PID Motor Status**: ACTIVE or STOPPED
- **PID RPM**: Target vs Current speed
- **PID Power**: Motor power level (0.0 to 1.0)
- **PID Control**: Reminder of which button to use

## 🔍 **Troubleshooting**

**Motor not responding?**
- Check that `pid_motor` is correctly configured in robot configuration
- Verify motor is connected to the correct port
- Check telemetry for "PID Motor: Not found" message

**RPM seems wrong?**
- The system assumes 1440 encoder counts per revolution (standard for goBILDA)
- If your motor has different encoder resolution, adjust the calculation in the code

**PID not stable?**
- Try adjusting PID constants as described above
- Ensure motor isn't mechanically loaded (free spinning for testing)

## 🚨 **Safety Notes**

- The motor will automatically stop when A button is released
- Maximum power is limited to 100% (1.0) for safety
- PID output is scaled down to prevent excessive power jumps
- Motor uses FLOAT zero power behavior for smooth operation