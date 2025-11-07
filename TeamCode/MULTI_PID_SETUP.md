# Multiple PID Motors Setup Guide

## 🔧 **Hardware Configuration Required**

Add these **three motors** to your robot configuration file:

### **Motor Names in Robot Config:**
1. **`launcher_motor`** - 5203 series 6000rpm goBILDA motor
2. **`pickup_motor`** - 5203 series 6000rpm goBILDA motor  
3. **`kicker_motor`** - 5203 series 312rpm goBILDA motor

### **Steps to Configure:**

1. **Connect Your Motors**
   - Connect each motor to available motor ports on your Control Hub/Expansion Hub
   - Note which ports you used for each motor

2. **Robot Configuration (Driver Station)**
   - Open the FTC Driver Station app
   - Tap **Configure** → Select your robot configuration
   - For each motor port, set the exact names shown above
   - Save the configuration

## 🎮 **Updated Control Layout**

| Button | Motor | RPM Target | Motor Type | Purpose |
|--------|-------|------------|------------|---------|
| **A** | Launcher | 4500 RPM | 5203-6000rpm | High-speed launching |
| **Y** | Pickup | 100 RPM | 5203-6000rpm | Slow pickup operations |
| **X** | Kicker | 150 RPM | 5203-312rpm | Precise kicking |

### **⚠️ Control Changes Made:**
- **Intake moved to DPad**: Left=forward, Right=reverse
- **Y and X buttons** now control PID motors
- **A button** changed from claw to launcher motor
- **Claw control** moved to right trigger (when pressed > 80%)

## ⚙️ **PID Tuning (Pre-configured)**

### **Launcher Motor (High Speed)**
- kP = 0.01, kI = 0.001, kD = 0.0001
- Optimized for 4500 RPM shooting

### **Pickup Motor (Low Speed)**  
- kP = 0.02, kI = 0.005, kD = 0.0001
- Higher gains for stable low-speed control

### **Kicker Motor (Ultra Precise)**
- kP = 0.05, kI = 0.01, kD = 0.0005
- Maximum precision for 150 RPM kicking tasks

## 📊 **Telemetry Display**

Real-time monitoring shows:
```
=== PID MOTORS ===
Launcher: ACTIVE | Target: 4500 | Current: 4485 | Power: 0.856
Pickup: STOPPED | Target: 100 | Current: 0 | Power: 0.000  
Kicker: STOPPED | Target: 150 | Current: 0 | Power: 0.000
PID Controls: A=Launcher(4500) Y=Pickup(100) X=Kicker(150)
```

## 🚀 **Usage Instructions**

1. **Deploy the updated code** to your robot
2. **Test each motor individually**:
   - Hold **A** → Launcher should spin at 4500 RPM
   - Hold **Y** → Pickup should spin at 100 RPM  
   - Hold **X** → Kicker should spin at 150 RPM
3. **Watch telemetry** for real-time feedback
4. **Release buttons** → Motors stop immediately

## 🔧 **Customization**

To change RPM targets, edit these lines in `MainTeleOpController.java`:

```java
private double launcherTargetRPM = 4500.0;   // A button target
private double pickupTargetRPM = 100.0;      // Y button target
private double kickerTargetRPM = 150.0;      // X button target
```

## 🎯 **Competition Ready Features**

✅ **Three independent PID systems**  
✅ **Real-time RPM monitoring**  
✅ **Instant safety shutoff**  
✅ **Optimized for goBILDA 5203 series**  
✅ **Professional telemetry display**  
✅ **Tuned PID constants included**  

## 🔍 **Troubleshooting**

### **"Not found - Feature disabled" in telemetry:**
- Check robot configuration motor names exactly match
- Verify motors are plugged into correct ports
- Restart robot and driver station

### **Motor spins but wrong speed:**
- Allow 2-3 seconds for PID to stabilize
- Check encoder cable connections
- Verify motor type matches configuration

### **Motor oscillates or unstable:**
- Mechanical binding - check for obstructions
- Reduce kP values if needed
- Ensure proper motor mounting

Your three-motor PID system is now ready for competition! 🏆