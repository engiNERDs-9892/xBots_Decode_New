# Updated Control Scheme - MainTeleOpController

## 🎮 Current Button Configuration

### **Primary Controls**
| Button | Function | Description |
|--------|----------|-------------|
| **A** | **🚀 Launch Sequence** | **Hold to activate launcher system with smart feeding** |
| **RB** | **🎯 Smart Pickup** | **Pickup + kicker with auto-stop on object detection** |
| **LB** | **🎯 Precision Mode** | **40% speed for precise movements** |
| **BACK** | **🧭 Reset Heading** | **Reset IMU field-centric orientation** |

### **Drive System**
| Control | Function | Speed |
|---------|----------|-------|
| **Left Stick** | Forward/Backward + Strafe | Variable |
| **Right Stick X** | Rotation | Variable |
| **Default** | Normal Mode | 80% power |
| **LB Held** | Precision Mode | 40% power |

### **Changes Made**
✅ **Moved precision mode** from Right Bumper → Left Bumper  
✅ **Removed turbo mode** completely (was Left Bumper)  
✅ **Right Bumper now available** for future features  

### **Available Buttons for Future Use**
- **B Button** - Available  
- **X Button** - Available
- **Y Button** - Available
- **START Button** - Available (field-centric toggle currently uses START)

## 🚀 Motor System Details

### **A Button - Launch Sequence (Complete Stop + Reset):**
1. **Press A** → **Immediately disables pickup** (safety override)
2. **Resets pickup kicker** → Re-enables kicker for future pickup cycles
3. **Launcher spins to 4500 RPM** → Pickup stays OFF during spin-up
4. **Within 2% tolerance** → Kicker (100 RPM) + Pickup (1000 RPM) activate
5. **Outside tolerance** → Feeding stops automatically
6. **Release A** → **ALL MOTORS STOP** (launcher, kicker, pickup)

### **RB Button - Smart Pickup Toggle:**
1. **Press RB** → Pickup (1000 RPM) + Kicker (100 RPM) start
2. **Color sensor monitors** → Detects green or purple objects
3. **Object detected** → Kicker stops automatically, pickup continues
4. **Kicker disabled** → Cannot restart until A button used
5. **Press RB again** → Turns off pickup system completely

### **Speed Control:**
- **Normal driving**: 80% power (default)
- **LB + driving**: 40% power (precision mode)
- **Works with all systems**: Can use precision mode while launching

## 📊 Telemetry Display

```
Speed Mode: NORMAL (0.8)  or  PRECISION (0.4)
=== COLOR SENSOR ===
Detected Color: Green
Pickup Target: DETECTED - Kicker stopped
=== LAUNCHER SYSTEM ===
Pickup Motor: Active: 1000 RPM (Toggle Active)
Kicker Motor: Disabled (Object detected - use A to reset)
Controls: A=Launch | RB=Smart Pickup
```

The control scheme is now simplified and optimized for competition use! 🏆