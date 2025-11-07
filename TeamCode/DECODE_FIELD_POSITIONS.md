# 🎯 DECODE Season (2025-2026) Field Coordinates

## 🏟️ **DECODE Field Layout - Starting Positions Against Scoring Wall**

For the **DECODE season (2025-2026)**, here are the coordinates for starting your robot against the scoring goal wall:

### 📐 **DECODE Field Coordinate System**

```
    Y-Axis (144")
         ↑
         |
      Scoring Goals
    [High] [Low] [High]
         |
         |
(0,0)————+————————————————→ X-Axis (144")
         |
         |
      Scoring Goals  
    [High] [Low] [High]
         |
         ↓
```

### 🎯 **Starting Positions Against Scoring Wall:**

#### **🔴 Red Alliance - Right Side High Goal:**
```java
// Starting against right high goal wall
setStartingPosition(60, 66, 180);  // 60" from center X, 66" from center Y, facing goals

// Alternative positions along the wall:
setStartingPosition(48, 66, 180);  // Closer to center
setStartingPosition(72, 66, 180);  // Further toward corner
```

#### **🔵 Blue Alliance - Left Side High Goal:**
```java
// Starting against left high goal wall  
setStartingPosition(-60, 66, 180); // -60" from center X, 66" from center Y, facing goals

// Alternative positions along the wall:
setStartingPosition(-48, 66, 180); // Closer to center
setStartingPosition(-72, 66, 180); // Further toward corner
```

#### **🎯 Center Low Goal Positions:**
```java
// Starting against center low goal wall
setStartingPosition(0, 66, 180);   // Directly in front of center low goal
setStartingPosition(-12, 66, 180); // Slightly left of center
setStartingPosition(12, 66, 180);  // Slightly right of center
```

### 🧭 **Starting Angles for DECODE:**

#### **📐 Angle Conventions:**
- **180°** = Facing the scoring goals (most common for preload scoring)
- **135°** = Angled toward high goal for better scoring approach
- **225°** = Angled away for quick sample collection after scoring

#### **🎯 Optimal Starting Angles:**
```java
// For immediate high goal scoring
setStartingPosition(60, 66, 135);   // Angled toward high goal

// For preload + quick collection strategy  
setStartingPosition(48, 66, 180);   // Straight facing goals

// For defensive/sample collection priority
setStartingPosition(72, 66, 225);   // Angled toward sample area
```

### 🚀 **Example DECODE Autonomous Sequences:**

#### **🎯 High Goal Preload Strategy:**
```java
// Start against high goal wall (Red Alliance example)
setStartingPosition(60, 66, 135);

// Score preload in high goal
moveToPosition(54, 60, 135);        // Approach high goal
executeAction(ACTION_LAUNCH_HIGH_POWER);  // Score with high power

// Move to sample collection
moveToPosition(24, 36, 90);         // Move to sample area
executeAction(ACTION_PICKUP_SEQUENCE);    // Collect samples

// Return to score samples
moveToPosition(54, 60, 135);        // Return to high goal
executeAction(ACTION_LAUNCH_HIGH_POWER);  // Score samples
```

#### **🔄 Low Goal Rush Strategy:**
```java
// Start against center low goal wall
setStartingPosition(0, 66, 180);

// Score preload in low goal
moveToPosition(0, 60, 180);         // Approach low goal
executeAction(ACTION_LAUNCH_STANDARD);    // Score with standard power

// Quick sample collection
moveToPosition(-30, 30, 270);       // Fast move to samples
executeAction(ACTION_PICKUP_SEQUENCE);    // Collect quickly

// Return to low goal
moveToPosition(0, 60, 180);         // Return to low goal
executeAction(ACTION_LAUNCH_STANDARD);    // Score samples
```

### 📏 **DECODE Field Specific Measurements:**

#### **🎯 Scoring Goal Positions (from field center):**
```java
// High Goals (estimated - verify with actual field)
private static final double RED_HIGH_GOAL_X = 54.0;
private static final double RED_HIGH_GOAL_Y = 60.0;
private static final double BLUE_HIGH_GOAL_X = -54.0;
private static final double BLUE_HIGH_GOAL_Y = 60.0;

// Low Goal (center)
private static final double CENTER_LOW_GOAL_X = 0.0;
private static final double CENTER_LOW_GOAL_Y = 60.0;

// Starting wall positions
private static final double SCORING_WALL_Y = 66.0;  // Distance from center to wall
```

#### **🔍 Sample Collection Areas (estimated):**
```java
// Sample areas (verify with actual field measurements)
private static final double SAMPLE_AREA_1_X = 36.0;
private static final double SAMPLE_AREA_1_Y = 24.0;

private static final double SAMPLE_AREA_2_X = -36.0;
private static final double SAMPLE_AREA_2_Y = 24.0;

private static final double CENTER_SAMPLES_X = 0.0;
private static final double CENTER_SAMPLES_Y = 0.0;
```

### ⚠️ **Important DECODE Season Notes:**

1. **Verify Measurements:** These coordinates are estimates - measure your actual field for precision
2. **Alliance Specific:** Red and Blue starting positions are mirrored
3. **Robot Size:** Account for your robot dimensions when positioning against walls
4. **Game Rules:** Check current game manual for legal starting positions
5. **Strategy:** Choose starting position based on your autonomous strategy

### 🔧 **Recommended Starting Position Setup:**

```java
// In your PositionSteeringAutonomous.java, update the starting position:

@Override
public void runOpMode() {
    // ... hardware initialization ...
    
    // DECODE Season - Starting against scoring wall
    // Choose based on your alliance and strategy:
    
    // Red Alliance High Goal Start:
    setStartingPosition(60, 66, 135);
    
    // Blue Alliance High Goal Start:
    // setStartingPosition(-60, 66, 135);
    
    // Center Low Goal Start:
    // setStartingPosition(0, 66, 180);
    
    // ... rest of autonomous sequence ...
}
```

### 🎯 **Testing Recommendations:**

1. **Measure your field** → Verify goal positions and wall distances
2. **Test starting position** → Ensure robot fits and is legal
3. **Calibrate coordinates** → Adjust based on actual measurements
4. **Practice sequences** → Test full autonomous from starting position

The key is measuring your specific DECODE field setup and adjusting these coordinates accordingly! 🤖🎯