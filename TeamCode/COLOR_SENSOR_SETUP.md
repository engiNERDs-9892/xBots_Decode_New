# REV Robotics Color Sensor V3 Setup Guide

## Hardware Configuration

### 1. Physical Connection
- Connect the REV Color Sensor V3 to an I2C port on your REV Control Hub or Expansion Hub
- The sensor uses the standard 4-wire I2C cable (JST PH 4-pin)
- Recommended: Use I2C port 0, 1, 2, or 3 on the Control Hub

### 2. Robot Configuration (Driver Station)
1. Open the **Robot Configuration** on your Driver Station
2. Navigate to the Control Hub configuration
3. Under **I2C Bus** section, add a new device:
   - **Device Type**: `REV Color/Range Sensor`
   - **Name**: `color_sensor` (exactly as used in code)
   - **I2C Address**: Usually `0x39` (default for REV Color Sensor V3)

### 3. Hardware Map Names
Both OpModes expect the color sensor to be configured with this exact name:
```
Hardware Map Name: color_sensor
Device Type: REV Color/Range Sensor
```

## Features Available

### MainTeleOpController.java
- **Color Detection**: Real-time RGB color readings
- **LED Control**: Toggle LED on/off with B button  
- **Basic Color Recognition**: Detects Red, Green, Blue, Black, White
- **Telemetry**: Shows RGB values and detected color

### PIDMotorTest.java  
- **Enhanced Color Testing**: Same features as main controller
- **LED Control**: Toggle LED with B button (Emergency stop moved to BACK button)
- **Color Telemetry**: Enhanced display with emoji indicators
- **Testing Environment**: Isolated color sensor testing

## Controls

### MainTeleOpController
- **B Button**: Toggle Color Sensor LED on/off

### PIDMotorTest  
- **B Button**: Toggle Color Sensor LED on/off
- **BACK Button**: Emergency stop all motors (changed from B button)

## Troubleshooting

### Sensor Not Found
If you see "Color Sensor: Not found" in telemetry:
1. Check physical I2C connection
2. Verify sensor is powered (LED should be visible if enabled)
3. Confirm hardware configuration name matches `color_sensor`
4. Try different I2C port if available
5. Check I2C address conflicts with other devices

### Poor Color Detection
1. Ensure adequate lighting or enable LED
2. Hold sensor 1-3 inches from surface
3. Use matte surfaces for better results
4. Adjust color detection thresholds in code if needed

### LED Not Working
1. Check if sensor supports LED (REV Color Sensor V3 does)
2. Verify LED is enabled in initialization
3. Try toggling with B button during OpMode

## Color Detection Thresholds

The current code uses these detection thresholds:
- **Red/Green/Blue**: > 0.3 individual channel value
- **Black**: Total RGB < 0.3  
- **White**: All RGB > 0.7

You can adjust these values in the telemetry section of each OpMode for your specific needs.

## Technical Notes

- **Sensor Type**: Uses `NormalizedColorSensor` interface for consistent 0.0-1.0 values
- **LED Control**: Implemented via `SwitchableLight` interface
- **Color Space**: Normalized RGBA values (Red, Green, Blue, Alpha)
- **Update Rate**: Real-time updates in main loop
- **Thread Safety**: No special threading considerations needed