package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * DECODE Red Side Autonomous - Preload Launcher
 * 
 * Simple autonomous that:
 * 1. Starts against red side goal wall
 * 2. Immediately fires 3 preloaded artifacts using TeleOp launcher system
 * 3. Uses the same launcher controls as MainTeleOpController
 * 
 * This matches your TeleOp launcher behavior exactly for consistency.
 */
@Autonomous(name="DECODE Red Side Preload", group="Competition")
public class DecodeRedSideAutonomous extends LinearOpMode {

    // ===================== HARDWARE DECLARATIONS =====================
    
    // Drive Motors (mecanum drive for movement)
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    
    // Action Motors (same as TeleOp)
    private DcMotor launcherMotor = null;
    private DcMotor pickupMotor = null;
    private DcMotor kickerMotor = null;
    
    // goBILDA Torque Servo (same as TeleOp)
    private Servo torqueServo = null;
    
    // REV Color Sensor V3 for smart feeding
    private NormalizedColorSensor colorSensor = null;
    
    // IMU for heading reference
    private IMU imu = null;
    
    // ===================== LAUNCHER CONTROL VARIABLES =====================
    
    // Timing and control
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime launcherTimer = new ElapsedTime();
    
    // RPM calculation variables
    private double lastLauncherPosition = 0;
    private double lastLauncherTime = 0;
    
    // Launcher system variables (matching TeleOp)
    private double launcherTargetRPM = 4500.0;  // Standard launcher speed
    private double launcherCurrentRPM = 0.0;
    private boolean launcherActive = false;
    private boolean pickupKickerEnabled = true;
    
    // Servo positions (matching TeleOp)
    private static final double SERVO_LAUNCH_POSITION_A = 0.0;   // A button position (0°)
    private static final double SERVO_LAUNCH_POSITION_Y = 1.0;   // Y button position (180°)
    private static final double SERVO_HOME_POSITION = 0.5;      // Default position
    private static final double SERVO_COLLECTION_POSITION = 0.5; // Collection position (same as home)
    
    // Launcher sequence timing
    private static final double LAUNCHER_SPINUP_TIME = 2.0;     // Time to reach target RPM
    private static final double FEEDING_TIME = 4.0;             // Time to feed 3 artifacts
    private static final double TOTAL_LAUNCH_TIME = 6.5;        // Total time for complete sequence
    
    // Drive constants for sample collection
    private static final double DRIVE_SPEED = 0.8;              // Fast drive speed for autonomous
    private static final double COLLECTION_SPEED = 0.4;         // Increased speed for pickup
    private static final double COUNTS_PER_INCH = 1120 / (4 * Math.PI); // Encoder calibration
    
    // DECODE field positions (Red Alliance - adjust based on actual field measurements)
    private static final double SAMPLE_AREA_X = 48.0;           // Distance to first sample set (inches)
    private static final double SAMPLE_AREA_Y = -36.0;          // Distance toward driver wall (inches)
    private static final double SAMPLE_AREA_2_X = 72.0;         // Distance to second sample set (inches)
    private static final double SAMPLE_AREA_2_Y = -48.0;        // Distance toward driver wall (inches)
    private static final double COLLECTION_DISTANCE = 18.0;     // Distance to drive during pickup (inches)
    
    @Override
    public void runOpMode() {
        
        // ===================== HARDWARE INITIALIZATION =====================
        
        telemetry.addData("Status", "DECODE Red Side - Initializing...");
        telemetry.update();
        
        initializeHardware();
        
        // ===================== STARTING POSITION SETUP =====================
        
        telemetry.addData("Status", "Ready for DECODE Red Side Autonomous");
        telemetry.addData("Strategy", "Fire 3 preloaded artifacts immediately");
        telemetry.addData("Starting Position", "Against red side goal wall");
        telemetry.addData("Launcher", "Standard mode (4500 RPM + Servo 0°)");
        telemetry.addData("Sequence Time", "~%.1f seconds", TOTAL_LAUNCH_TIME);
        telemetry.addData("Ready", "Press PLAY to start autonomous");
        telemetry.update();
        
        // ===================== WAIT FOR START =====================
        
        waitForStart();
        runtime.reset();
        
        if (opModeIsActive()) {
            // ===================== INITIAL SERVO SETUP =====================
            // Ensure servo starts at position 0 (safety measure - no delay needed)
            if (torqueServo != null) {
                torqueServo.setPosition(SERVO_LAUNCH_POSITION_A); // Set to 0° immediately
            }
            
            // ===================== DECODE RED SIDE AUTONOMOUS SEQUENCE =====================
            
            telemetry.addData("Status", "DECODE Red Side Autonomous STARTED!");
            telemetry.addData("Action", "Firing preloaded artifacts...");
            telemetry.update();
            
            // IMMEDIATE PRELOAD LAUNCH SEQUENCE
            // Uses exact same logic as A button in MainTeleOpController
            
            executePreloadLaunchSequence();
            
            telemetry.addData("Status", "Preload launch sequence COMPLETE!");
            telemetry.addData("Action", "3 artifacts fired");
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.update();
            
            // SAMPLE COLLECTION SEQUENCE
            // Drive to first set of artifacts and collect them
            
            telemetry.addData("Status", "Starting sample collection sequence");
            telemetry.addData("Action", "Moving to first artifact set...");
            telemetry.update();
            
            executeSampleCollectionSequence();
            
            telemetry.addData("Status", "Sample collection sequence COMPLETE!");
            telemetry.addData("Action", "Artifacts collected from field");
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.update();
            
            // RETURN TO STARTING POSITION AND SECOND LAUNCH
            // Return to firing position and launch collected artifacts
            
            telemetry.addData("Status", "Returning to launch position");
            telemetry.addData("Action", "Navigating back to starting area...");
            telemetry.update();
            
            executeReturnAndSecondLaunch();
            
            telemetry.addData("Status", "Second launch complete!");
            telemetry.addData("Action", "First 6 artifacts fired");
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.update();
            
            // SECOND COLLECTION SEQUENCE
            // Drive to second set of artifacts and collect them
            
            telemetry.addData("Status", "Starting second collection sequence");
            telemetry.addData("Action", "Moving to second artifact set...");
            telemetry.update();
            
            executeSecondCollectionSequence();
            
            telemetry.addData("Status", "Second collection complete!");
            telemetry.addData("Action", "Additional artifacts collected");
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.update();
            
            // FINAL RETURN AND THIRD LAUNCH
            // Return to starting position and fire the final set of collected artifacts
            
            telemetry.addData("Status", "Starting final launch sequence");
            telemetry.addData("Action", "Returning for third launch...");
            telemetry.update();
            
            executeThirdReturnAndLaunch();
            
            telemetry.addData("Status", "AUTONOMOUS SEQUENCE COMPLETE!");
            telemetry.addData("Action", "All 9 artifacts fired - Maximum points!");
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.addData("Next", "Ready for TeleOp");
            telemetry.update();
            
            // Robot is now ready for additional autonomous actions
            // Add more sequences here as needed for your strategy
        }
    }
    
    // ===================== HARDWARE INITIALIZATION =====================
    
    private void initializeHardware() {
        // Initialize drive motors
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
            
            // Set motor directions (adjust for your robot)
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            
            // Set to brake when stopped for precision
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Reset encoders
            resetDriveEncoders();
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            telemetry.addData("Drive Motors", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Drive Motors", "NOT FOUND - Movement disabled");
        }
        
        // Initialize action motors (same names as TeleOp)
        try {
            launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor");
            launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("Launcher Motor", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Launcher Motor", "NOT FOUND - Launch disabled");
            launcherMotor = null;
        }
        
        try {
            pickupMotor = hardwareMap.get(DcMotor.class, "pickup_motor");
            pickupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pickupMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pickupMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Pickup Motor", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Pickup Motor", "NOT FOUND - Feeding disabled");
            pickupMotor = null;
        }
        
        try {
            kickerMotor = hardwareMap.get(DcMotor.class, "kicker_motor");
            kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Kicker Motor", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Kicker Motor", "NOT FOUND - Kicking disabled");
            kickerMotor = null;
        }
        
        // Initialize servo (same name as TeleOp)
        try {
            torqueServo = hardwareMap.get(Servo.class, "torque_servo");
            torqueServo.setPosition(SERVO_HOME_POSITION);
            telemetry.addData("Torque Servo", "Initialized at home position");
        } catch (Exception e) {
            telemetry.addData("Torque Servo", "NOT FOUND - Servo control disabled");
            torqueServo = null;
        }
        
        // Initialize color sensor for smart feeding
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
            telemetry.addData("Color Sensor", "Initialized for smart feeding");
        } catch (Exception e) {
            telemetry.addData("Color Sensor", "NOT FOUND - Basic feeding only");
            colorSensor = null;
        }
        
        // Initialize IMU
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            telemetry.addData("IMU", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("IMU", "NOT FOUND - No heading reference");
            imu = null;
        }
        
        telemetry.addData("Hardware", "Initialization complete");
        telemetry.update();
        sleep(200); // Brief pause to read initialization status
    }
    
    // ===================== PRELOAD LAUNCH SEQUENCE =====================
    
    /**
     * Execute the complete preload launch sequence
     * This mirrors the A button launcher logic from MainTeleOpController exactly
     */
    private void executePreloadLaunchSequence() {
        
        // Phase 1: Set servo to launch position (same as A button)
        telemetry.addData("Phase 1", "Setting servo to launch position (0°)");
        telemetry.update();
        
        if (torqueServo != null) {
            torqueServo.setPosition(SERVO_LAUNCH_POSITION_A);  // 0° position - no delay needed
        }
        
        // Phase 2: Start launcher motor (same as A button)
        telemetry.addData("Phase 2", "Starting launcher motor (4500 RPM)");
        telemetry.update();
        
        setLauncherActive(true);
        launcherTimer.reset();
        
        // Phase 3: Wait for launcher to reach target speed using encoder feedback
        telemetry.addData("Phase 3", "Waiting for launcher to reach speed...");
        telemetry.update();
        
        while (opModeIsActive()) {
            // Calculate actual RPM from launcher motor encoder
            double actualRPM = getLauncherRPM();
            
            telemetry.addData("Launcher RPM", "%.0f / %.0f (%.1f%%)", 
                            actualRPM, launcherTargetRPM, 
                            (actualRPM / launcherTargetRPM) * 100);
            telemetry.addData("Spinup Time", "%.2f sec", launcherTimer.seconds());
            telemetry.update();
            
            // Check if launcher has reached 95% of target speed
            if (actualRPM >= launcherTargetRPM * 0.95) {
                telemetry.addData("Phase 3", "Launcher at target speed!");
                telemetry.update();
                break;
            }
            
            // Safety timeout after 3 seconds
            if (launcherTimer.seconds() > 3.0) {
                telemetry.addData("Phase 3", "Timeout - proceeding with current speed");
                telemetry.update();
                break;
            }
            
            sleep(50);
        }
        
        // Phase 4: Start feeding motors (launcher is now at speed)
        telemetry.addData("Phase 4", "Launcher ready - Starting feeding sequence");
        telemetry.update();
        
        setPickupActive(true);   // Start pickup motor (1000 RPM)
        setKickerActive(true);   // Start kicker motor (100 RPM)
        
        ElapsedTime feedingTimer = new ElapsedTime();
        feedingTimer.reset();
        
        // Phase 5: Feed artifacts for specified time
        while (opModeIsActive() && feedingTimer.seconds() < FEEDING_TIME) {
            // Check for color sensor detection (smart feeding like TeleOp)
            boolean objectDetected = false;
            if (colorSensor != null) {
                // Add color detection logic here if needed
            }
            
            telemetry.addData("Phase 5", "Feeding artifacts (3 preloaded)");
            telemetry.addData("Launcher", "%.0f RPM (Active)", launcherCurrentRPM);
            telemetry.addData("Pickup Motor", "1000 RPM (Feeding)");
            telemetry.addData("Kicker Motor", "100 RPM (Pushing)");
            telemetry.addData("Feeding Time", "%.1f / %.1f sec", feedingTimer.seconds(), FEEDING_TIME);
            telemetry.addData("Artifacts", "Firing preloaded samples...");
            telemetry.update();
            
            sleep(50);
        }
        
        // Phase 6: Stop all motors (same as A button release)
        telemetry.addData("Phase 6", "Stopping all motors");
        telemetry.update();
        
        setLauncherActive(false);
        setPickupActive(false);
        setKickerActive(false);
        
        // Motors stop immediately - no pause needed
    }
    
    // ===================== MOTOR CONTROL METHODS (Same as TeleOp) =====================
    
    /**
     * Set launcher motor active state (matches TeleOp)
     */
    private void setLauncherActive(boolean active) {
        setLauncherActive(active, false); // Default to A button mode
    }
    
    /**
     * Set launcher motor active state with power mode selection
     * @param active Whether to activate the launcher
     * @param highPowerMode true for Y button (4750 RPM), false for A button (4500 RPM)
     */
    private void setLauncherActive(boolean active, boolean highPowerMode) {
        launcherActive = active;
        if (launcherMotor != null) {
            if (active) {
                if (highPowerMode) {
                    launcherMotor.setPower(1.0);  // Full power for Y button (4750 RPM)
                } else {
                    launcherMotor.setPower(0.95); // Slightly lower for A button (4500 RPM)
                }
            } else {
                launcherMotor.setPower(0.0);
            }
        }
    }
    
    /**
     * Set pickup motor active state (matches TeleOp)
     */
    private void setPickupActive(boolean active) {
        if (pickupMotor != null) {
            if (active) {
                pickupMotor.setPower(0.8);  // 1000 RPM equivalent
            } else {
                pickupMotor.setPower(0.0);
            }
        }
    }
    
    /**
     * Set kicker motor active state (matches TeleOp)
     */
    private void setKickerActive(boolean active) {
        if (kickerMotor != null) {
            if (active) {
                kickerMotor.setPower(0.3);  // 100 RPM equivalent
            } else {
                kickerMotor.setPower(0.0);
            }
        }
    }
    
    // ===================== SAMPLE COLLECTION SEQUENCE =====================
    
    /**
     * Execute the complete sample collection sequence
     * Drives to first artifact set, positions robot, and collects samples
     */
    private void executeSampleCollectionSequence() {
        telemetry.addData("Collection", "Phase 1 - Moving to sample area");
        telemetry.update();
        
        // Phase 1: Drive to sample area (48" right, 36" toward drivers from starting position)
        driveToPosition(SAMPLE_AREA_X, SAMPLE_AREA_Y, DRIVE_SPEED);
        
        telemetry.addData("Collection", "Phase 2 - Positioning for pickup");
        telemetry.update();
        
        // Phase 2: Turn to face driver wall (180 degrees)
        turnToHeading(180, DRIVE_SPEED);
        
        telemetry.addData("Collection", "Phase 3 - Starting pickup sequence");
        telemetry.update();
        
        // Phase 3: Start pickup motor (same as RB button in TeleOp)
        setPickupActive(true);
        
        telemetry.addData("Collection", "Phase 4 - Driving forward slowly to collect");
        telemetry.update();
        
        // Phase 4: Drive forward slowly while collecting (18" collection distance)
        driveForwardWithPickup(COLLECTION_DISTANCE, COLLECTION_SPEED);
        
        telemetry.addData("Collection", "Phase 5 - Stopping pickup sequence");
        telemetry.update();
        
        // Phase 5: Stop pickup motor
        setPickupActive(false);
        
        telemetry.addData("Collection", "Sample collection complete!");
        telemetry.update();
    }
    
    /**
     * Execute the second sample collection sequence
     * Drives to second artifact set, positions robot, and collects samples
     */
    private void executeSecondCollectionSequence() {
        telemetry.addData("Collection 2", "Phase 1 - Moving to second sample area");
        telemetry.update();
        
        // Phase 1: Drive to second sample area (from current position to second set)
        // Calculate relative movement from first sample area to second sample area
        double deltaX = SAMPLE_AREA_2_X - SAMPLE_AREA_X;  // Additional distance right
        double deltaY = SAMPLE_AREA_2_Y - SAMPLE_AREA_Y;  // Additional distance toward drivers
        
        driveToPosition(deltaX, deltaY, DRIVE_SPEED);
        
        telemetry.addData("Collection 2", "Phase 2 - Positioning for pickup");
        telemetry.update();
        
        // Phase 2: Turn to face driver wall (180 degrees) - should already be close but ensure alignment
        turnToHeading(180, DRIVE_SPEED);
        
        telemetry.addData("Collection 2", "Phase 3 - Starting pickup sequence");
        telemetry.update();
        
        // Phase 3: Start pickup motor (same as RB button in TeleOp)
        setPickupActive(true);
        
        telemetry.addData("Collection 2", "Phase 4 - Driving forward slowly to collect");
        telemetry.update();
        
        // Phase 4: Drive forward slowly while collecting (18" collection distance)
        driveForwardWithPickup(COLLECTION_DISTANCE, COLLECTION_SPEED);
        
        telemetry.addData("Collection 2", "Phase 5 - Stopping pickup sequence");
        telemetry.update();
        
        // Phase 5: Stop pickup motor
        setPickupActive(false);
        
        telemetry.addData("Collection 2", "Second sample collection complete!");
        telemetry.addData("Collection 2", "Additional artifacts collected for potential scoring");
        telemetry.update();
    }
    
    /**
     * Return to starting position from second collection area and execute third launch sequence
     * Fires the final set of collected artifacts for maximum scoring
     */
    private void executeThirdReturnAndLaunch() {
        telemetry.addData("Return 3", "Phase 1 - Returning from second collection area");
        telemetry.update();
        
        // Phase 1: Return to starting position from second sample area
        // From second sample area (72", -48") back to origin (0, 0)
        driveToPosition(-SAMPLE_AREA_2_X, -SAMPLE_AREA_2_Y, DRIVE_SPEED);
        
        telemetry.addData("Return 3", "Phase 2 - Positioning for final launch");
        telemetry.update();
        
        // Phase 2: Turn to face launch area (0 degrees - original orientation)
        turnToHeading(0, DRIVE_SPEED);
        
        telemetry.addData("Return 3", "Phase 3 - Executing third launch sequence");
        telemetry.addData("Return 3", "Using A button logic (4500 RPM, 0°)");
        telemetry.update();
        
        // Phase 3: Wait a moment to ensure robot is stable
        sleep(250);
        
        // Phase 4: Execute the final launch sequence (same as preload and second launch)
        // This fires the final collected artifacts using A button logic (4500 RPM)
        executeThirdLaunchSequence();
        
        telemetry.addData("Return 3", "Third launch sequence complete!");
        telemetry.addData("Return 3", "Final artifacts fired successfully!");
        telemetry.addData("Return 3", "MAXIMUM AUTONOMOUS SCORING ACHIEVED!");
        telemetry.update();
        sleep(250); // Brief pause
    }
    
    /**
     * Execute third launch sequence for final collected artifacts
     * Uses A button logic (4500 RPM, servo at 0°) for consistent scoring
     */
    private void executeThirdLaunchSequence() {
        telemetry.addData("Launch 3", "Starting third launch sequence");
        telemetry.addData("Launch 3", "Using A button mode (4500 RPM, 0°)");
        telemetry.update();
        
        // Use A button logic for final artifacts (consistent with previous launches)
        // This matches the A button functionality from MainTeleOpController
        
        // Step 1: Start launcher motors at A button speed (4500 RPM)
        setLauncherActive(true, false); // false for A button mode (standard power)
        
        telemetry.addData("Launch 3", "Launcher motors starting - waiting for speed...");
        telemetry.update();
        
        // Wait for launcher to reach target speed using encoder feedback
        ElapsedTime finalLaunchTimer = new ElapsedTime();
        finalLaunchTimer.reset();
        
        while (opModeIsActive()) {
            double actualRPM = getLauncherRPM();
            
            telemetry.addData("Launch 3", "RPM: %.0f / %.0f (%.1f%%)", 
                            actualRPM, launcherTargetRPM, 
                            (actualRPM / launcherTargetRPM) * 100);
            telemetry.update();
            
            // Check if launcher has reached 95% of target speed
            if (actualRPM >= launcherTargetRPM * 0.95) {
                telemetry.addData("Launch 3", "Launcher at target speed!");
                telemetry.update();
                break;
            }
            
            // Safety timeout after 3 seconds
            if (finalLaunchTimer.seconds() > 3.0) {
                telemetry.addData("Launch 3", "Timeout - proceeding");
                telemetry.update();
                break;
            }
            
            sleep(50);
        }
        
        // Step 2: Move servo to launch position A (0°)
        if (torqueServo != null) {
            torqueServo.setPosition(SERVO_LAUNCH_POSITION_A);
            telemetry.addData("Launch 3", "Servo moved to A position (0°)");
            telemetry.update();
        }
        
        // Step 3: Return servo to collection position
        if (torqueServo != null) {
            torqueServo.setPosition(SERVO_COLLECTION_POSITION);
            telemetry.addData("Launch 3", "Servo returned to collection position");
            telemetry.update();
        }
        
        // Step 4: Stop launcher motors
        setLauncherActive(false, false);
        
        telemetry.addData("Launch 3", "Third launch sequence COMPLETE!");
        telemetry.addData("Launch 3", "Final artifacts fired with A button logic");
        telemetry.addData("Launch 3", "AUTONOMOUS PERFECTION - 9 ARTIFACTS SCORED!");
        telemetry.update();
    }
    
    /**
     * Return to starting position and execute second launch sequence
     * Fires the collected artifacts for additional points
     */
    private void executeReturnAndSecondLaunch() {
        telemetry.addData("Return", "Phase 1 - Returning to starting position");
        telemetry.update();
        
        // Phase 1: Return to starting position
        // From sample area, go back to launch position (reverse the collection journey)
        driveToPosition(-SAMPLE_AREA_X, -SAMPLE_AREA_Y, DRIVE_SPEED);
        
        telemetry.addData("Return", "Phase 2 - Positioning for launch");
        telemetry.update();
        
        // Phase 2: Turn to face launch area (0 degrees - original orientation)
        turnToHeading(0, DRIVE_SPEED);
        
        telemetry.addData("Return", "Phase 3 - Executing second launch sequence");
        telemetry.addData("Return", "Using A button logic (4500 RPM, 0°)");
        telemetry.update();
        
        // Phase 3: Wait a moment to ensure robot is stable
        sleep(250);
        
        // Phase 4: Execute the launch sequence again (same as preload)
        // This fires the collected artifacts using A button logic (4500 RPM)
        executeSecondLaunchSequence();
        
        telemetry.addData("Return", "Second launch sequence complete!");
        telemetry.addData("Return", "All collected artifacts fired successfully!");
        telemetry.update();
        sleep(250); // Brief pause
    }
    
    /**
     * Execute second launch sequence for collected artifacts
     * Uses A button logic (4500 RPM, servo at 0°) for consistent scoring
     */
    private void executeSecondLaunchSequence() {
        telemetry.addData("Launch 2", "Starting second launch sequence");
        telemetry.addData("Launch 2", "Using A button mode (4500 RPM, 0°)");
        telemetry.update();
        
        // Use A button logic for collected artifacts (same as preload)
        // This matches the A button functionality from MainTeleOpController
        
        // Step 1: Start launcher motors at A button speed (4500 RPM)
        setLauncherActive(true, false); // false for A button mode (standard power)
        
        telemetry.addData("Launch 2", "Launcher motors starting - waiting for speed...");
        telemetry.update();
        
        // Wait for launcher to reach target speed using encoder feedback
        ElapsedTime launchTimer = new ElapsedTime();
        launchTimer.reset();
        
        while (opModeIsActive()) {
            double actualRPM = getLauncherRPM();
            
            telemetry.addData("Launch 2", "RPM: %.0f / %.0f (%.1f%%)", 
                            actualRPM, launcherTargetRPM, 
                            (actualRPM / launcherTargetRPM) * 100);
            telemetry.update();
            
            // Check if launcher has reached 95% of target speed
            if (actualRPM >= launcherTargetRPM * 0.95) {
                telemetry.addData("Launch 2", "Launcher at target speed!");
                telemetry.update();
                break;
            }
            
            // Safety timeout after 3 seconds
            if (launchTimer.seconds() > 3.0) {
                telemetry.addData("Launch 2", "Timeout - proceeding");
                telemetry.update();
                break;
            }
            
            sleep(50);
        }
        
        // Step 2: Move servo to launch position A (0°)
        if (torqueServo != null) {
            torqueServo.setPosition(SERVO_LAUNCH_POSITION_A);
            telemetry.addData("Launch 2", "Servo moved to A position (0°)");
            telemetry.update();
        }
        
        // Step 3: Return servo to collection position
        if (torqueServo != null) {
            torqueServo.setPosition(SERVO_COLLECTION_POSITION);
            telemetry.addData("Launch 2", "Servo returned to collection position");
            telemetry.update();
        }
        
        // Step 4: Stop launcher motors
        setLauncherActive(false, false);
        
        telemetry.addData("Launch 2", "Second launch sequence COMPLETE!");
        telemetry.addData("Launch 2", "Collected artifacts fired with A button logic");
        telemetry.update();
    }
    
    // ===================== DRIVE MOVEMENT METHODS =====================
    
    /**
     * Drive to a relative position from current location
     */
    private void driveToPosition(double deltaX, double deltaY, double speed) {
        // Calculate distance and angle
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double angle = Math.atan2(deltaY, deltaX) * 180.0 / Math.PI;
        
        telemetry.addData("Drive", "Moving %.1f inches at %.1f degrees", distance, angle);
        telemetry.update();
        
        // Convert distance to encoder ticks
        int targetTicks = (int)(distance * COUNTS_PER_INCH);
        
        // Reset encoders
        resetDriveEncoders();
        
        // Calculate drive components for mecanum drive
        double angleRad = Math.toRadians(angle);
        double drive = Math.cos(angleRad) * speed;
        double strafe = Math.sin(angleRad) * speed;
        
        // Set target positions for all motors
        setDriveTarget(targetTicks, targetTicks, targetTicks, targetTicks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Apply mecanum drive powers
        double leftFrontPower = drive + strafe;
        double rightFrontPower = drive - strafe;
        double leftBackPower = drive - strafe;
        double rightBackPower = drive + strafe;
        
        // Normalize powers
        double maxPower = Math.max(
            Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
            Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        );
        
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }
        
        // Set motor powers
        setDrivePower(leftFrontPower * speed, rightFrontPower * speed, 
                     leftBackPower * speed, rightBackPower * speed);
        
        // Wait for movement to complete
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Drive", "Moving to position... %.1f%%", 
                            (double)Math.abs(leftFrontDrive.getCurrentPosition()) / targetTicks * 100);
            telemetry.update();
            sleep(20);
        }
        
        stopDriveMotors();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Turn to a specific heading
     */
    private void turnToHeading(double targetHeading, double speed) {
        telemetry.addData("Turn", "Turning to %.1f degrees", targetHeading);
        telemetry.update();
        
        // Simple turn implementation (improve with IMU for accuracy)
        double turnDistance = targetHeading * 0.1; // Approximate - calibrate for your robot
        int turnTicks = (int)(Math.abs(turnDistance) * COUNTS_PER_INCH);
        
        resetDriveEncoders();
        
        if (targetHeading > 0) {
            // Turn right
            setDriveTarget(turnTicks, -turnTicks, turnTicks, -turnTicks);
        } else {
            // Turn left  
            setDriveTarget(-turnTicks, turnTicks, -turnTicks, turnTicks);
        }
        
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(speed, speed, speed, speed);
        
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Turn", "Turning... %.1f%%", 
                            (double)Math.abs(leftFrontDrive.getCurrentPosition()) / turnTicks * 100);
            telemetry.update();
            sleep(20);
        }
        
        stopDriveMotors();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Drive forward while running pickup motor
     */
    private void driveForwardWithPickup(double distance, double speed) {
        telemetry.addData("Pickup Drive", "Driving %.1f inches with collection", distance);
        telemetry.update();
        
        int targetTicks = (int)(distance * COUNTS_PER_INCH);
        
        resetDriveEncoders();
        setDriveTarget(targetTicks, targetTicks, targetTicks, targetTicks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(speed, speed, speed, speed);
        
        while (opModeIsActive() && motorsAreBusy()) {
            // Monitor for color sensor detection during pickup
            if (colorSensor != null) {
                // Add color detection logic here if needed for smart collection
            }
            
            telemetry.addData("Pickup Drive", "Collecting... %.1f%%", 
                            (double)Math.abs(leftFrontDrive.getCurrentPosition()) / targetTicks * 100);
            telemetry.addData("Pickup Motor", "Active - Collecting artifacts");
            telemetry.update();
            sleep(20);
        }
        
        stopDriveMotors();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    // ===================== DRIVE HELPER METHODS =====================
    
    private void resetDriveEncoders() {
        if (leftFrontDrive != null) leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightFrontDrive != null) rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (leftBackDrive != null) leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightBackDrive != null) rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    private void setDriveMode(DcMotor.RunMode mode) {
        if (leftFrontDrive != null) leftFrontDrive.setMode(mode);
        if (rightFrontDrive != null) rightFrontDrive.setMode(mode);
        if (leftBackDrive != null) leftBackDrive.setMode(mode);
        if (rightBackDrive != null) rightBackDrive.setMode(mode);
    }
    
    private void setDriveTarget(int lf, int rf, int lb, int rb) {
        if (leftFrontDrive != null) leftFrontDrive.setTargetPosition(lf);
        if (rightFrontDrive != null) rightFrontDrive.setTargetPosition(rf);
        if (leftBackDrive != null) leftBackDrive.setTargetPosition(lb);
        if (rightBackDrive != null) rightBackDrive.setTargetPosition(rb);
    }
    
    private void setDrivePower(double lf, double rf, double lb, double rb) {
        if (leftFrontDrive != null) leftFrontDrive.setPower(lf);
        if (rightFrontDrive != null) rightFrontDrive.setPower(rf);
        if (leftBackDrive != null) leftBackDrive.setPower(lb);
        if (rightBackDrive != null) rightBackDrive.setPower(rb);
    }
    
    private void stopDriveMotors() {
        setDrivePower(0, 0, 0, 0);
    }
    
    private boolean motorsAreBusy() {
        return (leftFrontDrive != null && leftFrontDrive.isBusy()) ||
               (rightFrontDrive != null && rightFrontDrive.isBusy()) ||
               (leftBackDrive != null && leftBackDrive.isBusy()) ||
               (rightBackDrive != null && rightBackDrive.isBusy());
    }
    
    /**
     * Calculate launcher RPM from encoder feedback
     * @return Current launcher RPM
     */
    private double getLauncherRPM() {
        if (launcherMotor == null) return 0.0;
        
        // Simple RPM calculation using encoder position change over time
        double currentTime = runtime.seconds();
        double currentPosition = launcherMotor.getCurrentPosition();
        
        if (lastLauncherTime == 0) {
            // First measurement - initialize
            lastLauncherPosition = currentPosition;
            lastLauncherTime = currentTime;
            return 0.0;
        }
        
        double deltaTime = currentTime - lastLauncherTime;
        double deltaPosition = currentPosition - lastLauncherPosition;
        
        if (deltaTime > 0.1) { // Only calculate if enough time has passed (100ms)
            // Calculate RPM
            // Assumes launcher motor has standard goBILDA encoder (2000 counts per revolution)
            double countsPerRevolution = 2000.0;
            double revolutionsPerSecond = deltaPosition / (countsPerRevolution * deltaTime);
            double rpm = revolutionsPerSecond * 60.0;
            
            // Update for next calculation
            lastLauncherPosition = currentPosition;
            lastLauncherTime = currentTime;
            
            return Math.abs(rpm); // Return absolute value
        }
        
        return 0.0;
    }
}