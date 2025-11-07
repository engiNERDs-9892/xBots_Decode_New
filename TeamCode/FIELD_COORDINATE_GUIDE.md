# 🗺️ FTC Field Coordinate System Guide

## 📐 **Coordinate System Explanation**

The position steering autonomous uses a **Cartesian coordinate system** overlaid on the FTC field. Here's how it works:

### 🎯 **Standard FTC Field Layout (12' x 12' = 144" x 144")**

```
    Y-Axis (144")
         ↑
         |
  Blue   |   Red
Alliance |  Alliance
         |
         |
(0,0)————+————————————————→ X-Axis (144")
         |
         |
  Blue   |   Red  
Alliance |  Alliance
         |
         ↓
```

### 📍 **Coordinate Origin Options:**

#### **Option 1: Field Center (Recommended)**
```java
// Field center as origin (72", 72")
setStartingPosition(0, 0, 0);  // Robot starts at field center

// Example positions:
moveToPosition(36, 36, 0);   // 36" right, 36" forward from center
moveToPosition(-24, 48, 90); // 24" left, 48" forward from center
moveToPosition(0, -60, 180); // On center line, 60" toward drivers
```

#### **Option 2: Corner Origin (Alternative)**
```java
// Corner as origin (0", 0") - bottom-left corner
setStartingPosition(72, 72, 0);  // Robot starts at field center

// Example positions:
moveToPosition(108, 108, 0);  // 108" from corner (field center + 36")
moveToPosition(36, 120, 90);  // Near scoring area
moveToPosition(120, 36, 180); // Opposite scoring area
```

### 🏟️ **Typical Game Element Positions (Example - Adjust for Current Game)**

#### **🎯 Scoring Areas:**
```java
// High scoring positions (adjust for current game)
private static final double HIGH_BASKET_X = 24.0;
private static final double HIGH_BASKET_Y = 48.0;
private static final double HIGH_BASKET_HEADING = 45.0;

// Low scoring positions
private static final double LOW_BASKET_X = -12.0;
private static final double LOW_BASKET_Y = 36.0;
private static final double LOW_BASKET_HEADING = 0.0;
```

#### **🔍 Sample Collection Areas:**
```java
// Sample pickup locations (adjust for current game)
private static final double SAMPLE_AREA_1_X = 48.0;
private static final double SAMPLE_AREA_1_Y = 12.0;
private static final double SAMPLE_AREA_1_HEADING = 90.0;

private static final double SAMPLE_AREA_2_X = -36.0;
private static final double SAMPLE_AREA_2_Y = -24.0;
private static final double SAMPLE_AREA_2_HEADING = 270.0;
```

#### **🏠 Parking/Safe Zones:**
```java
// Parking areas (adjust for current game)
private static final double PARKING_ZONE_X = 60.0;
private static final double PARKING_ZONE_Y = -60.0;
private static final double PARKING_ZONE_HEADING = 180.0;
```

### 🧭 **Heading Convention:**

```
       0° (North)
         ↑
         |
270° ←——— ———→ 90° (East)
(West)   |
         ↓
      180° (South)
```

- **0°** = Facing away from drivers (toward opposite alliance)
- **90°** = Facing right side of field
- **180°** = Facing toward drivers (toward your alliance)
- **270°** = Facing left side of field

### 🎮 **Real-World Field Mapping Process:**

#### **Step 1: Choose Your Origin**
```java
// Recommended: Use field center as origin for symmetrical coordinates
setStartingPosition(0, 0, 0);  // Field center

// Alternative: Use your starting tile corner
setStartingPosition(-60, -60, 0);  // Corner of starting tile
```

#### **Step 2: Measure Key Positions**
```java
// Physically measure distances from your origin to:
// 1. Scoring baskets/goals
// 2. Sample pickup areas  
// 3. Parking zones
// 4. Any obstacle positions

// Example measurements (replace with actual):
moveToPosition(28, 42, 45);   // High basket (measured from center)
moveToPosition(-18, 36, 0);   // Low basket (measured from center)
moveToPosition(54, 18, 90);   // Sample area (measured from center)
```

#### **Step 3: Test and Verify**
```java
// Create test sequence to verify coordinates:
public void testFieldMapping() {
    // Move to known field feature
    moveToPosition(MEASURED_X, MEASURED_Y, MEASURED_HEADING);
    
    // Stop and verify robot is at correct location
    executeAction(ACTION_WAIT);
    
    // Move to next known feature
    moveToPosition(NEXT_X, NEXT_Y, NEXT_HEADING);
}
```

### 🔧 **Limelight Integration for Accuracy**

#### **AprilTag Positions:**
```java
// FTC fields typically have AprilTags at known positions
// Use Limelight to detect these for position correction

// Example AprilTag positions (check current game manual):
// Tag ID 1: Position (0, 72, 0)     - Field center wall
// Tag ID 2: Position (72, 144, 90)  - Corner position  
// Tag ID 3: Position (-72, 0, 270)  - Side wall position
```

#### **Automatic Position Correction:**
```java
// In updatePositionFromLimelight():
// When Limelight sees an AprilTag, correct robot position
if (tagDetected) {
    // Use known tag position to update currentX, currentY, currentHeading
    currentX = calculatedXFromTag;
    currentY = calculatedYFromTag;  
    currentHeading = calculatedHeadingFromTag;
}
```

### 📏 **Calibration Example for Current Season:**

#### **Step-by-Step Field Mapping:**
```java
// 1. Place robot at field center, facing away from drivers
setStartingPosition(0, 0, 0);

// 2. Manually drive to scoring area and note coordinates
// Example: High basket is 30" right, 45" forward from center
private static final double HIGH_SCORING_X = 30.0;
private static final double HIGH_SCORING_Y = 45.0;

// 3. Map other key positions similarly
private static final double SAMPLE_PICKUP_X = -42.0;
private static final double SAMPLE_PICKUP_Y = 18.0;

// 4. Use in autonomous:
moveToPosition(HIGH_SCORING_X, HIGH_SCORING_Y, 45);
executeAction(ACTION_LAUNCH_HIGH_POWER);
```

### 🎯 **Competition Strategy Coordinates:**

#### **Example Multi-Point Autonomous:**
```java
// Starting position (adjust for your alliance and starting tile)
setStartingPosition(-48, -48, 45);  // Corner tile, angled start

// Sequence 1: Score preload
moveToPosition(-12, 24, 0);         // Move to scoring position
executeAction(ACTION_LAUNCH_STANDARD);

// Sequence 2: Collect samples  
moveToPosition(36, -12, 90);        // Move to sample area
executeAction(ACTION_PICKUP_SEQUENCE);

// Sequence 3: Score collected samples
moveToPosition(18, 48, 135);        // Move to high scoring position
executeAction(ACTION_LAUNCH_HIGH_POWER);

// Sequence 4: Park
moveToPosition(60, -60, 180);       // Move to parking area
```

### ⚠️ **Important Notes:**

1. **Game-Specific:** Coordinates must be updated each season based on current game elements
2. **Alliance-Specific:** Red and blue alliance may need different coordinate sets
3. **Robot Size:** Account for robot dimensions when positioning near walls/obstacles
4. **Testing Required:** Always test coordinates on actual field before competition
5. **Limelight Calibration:** Set up AprilTag detection for automatic position correction

### 🔧 **Quick Setup for Current Game:**

1. **📏 Measure your field** → Find distances to key game elements
2. **🎯 Choose origin** → Field center recommended for symmetry  
3. **📝 Create constants** → Define positions as named constants
4. **🧪 Test movement** → Verify robot reaches correct locations
5. **🎥 Add Limelight** → Use AprilTags for position correction
6. **🏆 Build strategy** → Chain movements for competition sequence

The coordinate system gives you precise control over robot positioning for optimal scoring and game strategy! 🤖🎯