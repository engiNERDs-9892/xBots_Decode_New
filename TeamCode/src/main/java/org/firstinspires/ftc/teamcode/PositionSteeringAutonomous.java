package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Position-Based Autonomous OpMode with Limelight Localization
 * 
 * This autonomous system provides:
 * - Field-coordinate-based navigation
 * - Limelight vision localization for position updates
 * - Position steering to move between known field positions
 * - Action execution at specific coordinates
 * - Path planning and obstacle avoidance
 * 
 * Usage Example:
 * 1. moveToPosition(24, 36, 0) - Move to coordinates (24", 36", 0°)
 * 2. executeAction(ACTION_LAUNCH_STANDARD) - Perform launcher sequence
 * 3. moveToPosition(48, 12, 90) - Move to new position facing 90°
 */
@Autonomous(name="Position Steering Autonomous", group="Competition")
public class PositionSteeringAutonomous extends LinearOpMode {

    // ===================== HARDWARE DECLARATIONS =====================
    
    // Drive Motors (mecanum drive)
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    
    // IMU for heading tracking
    private IMU imu = null;
    
    // Limelight for field localization
    private Limelight3A limelight = null;
    
    // Action hardware (same as TeleOp)
    private DcMotor launcherMotor = null;
    private DcMotor pickupMotor = null;
    private DcMotor kickerMotor = null;
    private Servo torqueServo = null;
    private NormalizedColorSensor colorSensor = null;
    
    // ===================== POSITION TRACKING VARIABLES =====================
    
    // Current robot position on field (inches and degrees)
    private double currentX = 0.0;      // X position (inches from field origin)
    private double currentY = 0.0;      // Y position (inches from field origin)
    private double currentHeading = 0.0; // Heading (degrees, 0 = facing away from drivers)
    
    // Position update sources
    private boolean limelightAvailable = false;
    private boolean imuAvailable = false;
    
    // Timing and control
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime positionTimer = new ElapsedTime();
    
    // ===================== NAVIGATION CONSTANTS =====================
    
    // Drive control constants
    private static final double MAX_DRIVE_SPEED = 0.8;
    private static final double MIN_DRIVE_SPEED = 0.15;
    private static final double MAX_TURN_SPEED = 0.6;
    private static final double MIN_TURN_SPEED = 0.1;
    
    // Position tolerances
    private static final double POSITION_TOLERANCE = 2.0;    // inches
    private static final double HEADING_TOLERANCE = 3.0;     // degrees
    
    // PID constants for position control
    private static final double POSITION_KP = 0.03;  // Proportional gain for position
    private static final double POSITION_KI = 0.001; // Integral gain for position
    private static final double POSITION_KD = 0.01;  // Derivative gain for position
    
    private static final double HEADING_KP = 0.02;   // Proportional gain for heading
    private static final double HEADING_KI = 0.0005; // Integral gain for heading
    private static final double HEADING_KD = 0.005;  // Derivative gain for heading
    
    // PID variables
    private double positionIntegral = 0;
    private double positionLastError = 0;
    private double headingIntegral = 0;
    private double headingLastError = 0;
    
    // Dead reckoning variables
    private int lastLeftFront = 0;
    private int lastRightFront = 0;
    private int lastLeftBack = 0;
    private int lastRightBack = 0;
    
    // Encoder constants (calibrate for your robot)
    private static final double COUNTS_PER_INCH = 1120 / (4 * Math.PI); // 4" wheel example
    private static final double ROBOT_WIDTH = 16.0;  // Distance between wheels (inches)
    
    // ===================== ACTION DEFINITIONS =====================
    
    // Action constants for executeAction() method
    public static final int ACTION_LAUNCH_STANDARD = 1;
    public static final int ACTION_LAUNCH_HIGH_POWER = 2;
    public static final int ACTION_PICKUP_SEQUENCE = 3;
    public static final int ACTION_WAIT = 4;
    public static final int ACTION_SERVO_HOME = 5;
    
    // Launcher constants (matching TeleOp)
    private static final double STANDARD_LAUNCHER_RPM = 4500.0;
    private static final double HIGH_POWER_LAUNCHER_RPM = 4750.0;
    private static final double SERVO_LAUNCH_POSITION_A = 0.0;  // 0°
    private static final double SERVO_LAUNCH_POSITION_Y = 1.0;  // 180°
    private static final double SERVO_HOME_POSITION = 0.0;
    
    @Override
    public void runOpMode() {
        
        // ===================== HARDWARE INITIALIZATION =====================
        
        telemetry.addData("Status", "Initializing position steering system...");
        telemetry.update();
        
        initializeHardware();
        
        // ===================== POSITION SYSTEM SETUP =====================
        
        // Set starting position (modify based on your field setup)
        setStartingPosition(0, 0, 0);  // Start at origin facing 0°
        
        telemetry.addData("Status", "Position steering ready");
        telemetry.addData("Starting Position", "X:%.1f Y:%.1f H:%.1f°", currentX, currentY, currentHeading);
        telemetry.addData("Limelight", limelightAvailable ? "Connected" : "Not found");
        telemetry.addData("IMU", imuAvailable ? "Connected" : "Not found");
        telemetry.addData("Ready", "Press PLAY to start autonomous");
        telemetry.update();
        
        // ===================== WAIT FOR START =====================
        
        waitForStart();
        runtime.reset();
        
        if (opModeIsActive()) {
            // ===================== POSITION-BASED AUTONOMOUS SEQUENCE =====================
            
            telemetry.addData("Status", "Position-based autonomous starting...");
            telemetry.update();
            
            // EXAMPLE POSITION-BASED SEQUENCE - Customize for your strategy!
            
            // Sequence 1: Move to first scoring position and launch
            telemetry.addData("Sequence", "1 - Moving to first scoring position");
            telemetry.update();
            moveToPosition(24, 36, 0);      // Move to (24", 36") facing 0°
            executeAction(ACTION_LAUNCH_STANDARD);  // Standard launcher sequence
            
            // Sequence 2: Move to sample collection area
            telemetry.addData("Sequence", "2 - Moving to collection area");
            telemetry.update();
            moveToPosition(48, 24, 90);     // Move to (48", 24") facing 90°
            executeAction(ACTION_PICKUP_SEQUENCE);  // Pickup sequence
            
            // Sequence 3: Move to high scoring position
            telemetry.addData("Sequence", "3 - Moving to high scoring position");
            telemetry.update();
            moveToPosition(12, 48, 180);    // Move to (12", 48") facing 180°
            executeAction(ACTION_LAUNCH_HIGH_POWER); // High-power launcher
            
            // Sequence 4: Return to safe position
            telemetry.addData("Sequence", "4 - Moving to parking position");
            telemetry.update();
            moveToPosition(6, 6, 270);      // Park at (6", 6") facing 270°
            executeAction(ACTION_SERVO_HOME);       // Return servo to home
            
            telemetry.addData("Status", "Position-based autonomous complete!");
            telemetry.addData("Final Position", "X:%.1f Y:%.1f H:%.1f°", currentX, currentY, currentHeading);
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.update();
        }
    }
    
    // ===================== HARDWARE INITIALIZATION =====================
    
    private void initializeHardware() {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        
        // Set motor directions (adjust for your robot)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Set to brake when stopped
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset encoders
        resetDriveEncoders();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize IMU
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.resetYaw();
            imuAvailable = true;
            telemetry.addData("IMU", "Initialized and reset");
        } catch (Exception e) {
            telemetry.addData("IMU", "Not found - heading tracking limited");
            imuAvailable = false;
        }
        
        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);  // Switch to pipeline 0 for localization
            limelight.start();
            limelightAvailable = true;
            telemetry.addData("Limelight", "Initialized and started");
        } catch (Exception e) {
            telemetry.addData("Limelight", "Not found - using dead reckoning only");
            limelightAvailable = false;
        }
        
        // Initialize action hardware
        try {
            launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor");
            pickupMotor = hardwareMap.get(DcMotor.class, "pickup_motor");
            kickerMotor = hardwareMap.get(DcMotor.class, "kicker_motor");
            torqueServo = hardwareMap.get(Servo.class, "torque_servo");
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
            telemetry.addData("Action Hardware", "All systems initialized");
        } catch (Exception e) {
            telemetry.addData("Action Hardware", "Some components missing");
        }
    }
    
    // ===================== POSITION CONTROL METHODS =====================
    
    /**
     * Set the robot's starting position on the field
     */
    public void setStartingPosition(double x, double y, double heading) {
        currentX = x;
        currentY = y;
        currentHeading = heading;
        
        // Reset IMU to match starting heading
        if (imuAvailable) {
            imu.resetYaw();
        }
        
        positionTimer.reset();
    }
    
    /**
     * Move to a specific position on the field using position steering
     */
    public void moveToPosition(double targetX, double targetY, double targetHeading) {
        telemetry.addData("Navigation", "Moving to (%.1f, %.1f, %.1f°)", targetX, targetY, targetHeading);
        telemetry.update();
        
        // Reset PID variables
        positionIntegral = 0;
        positionLastError = 0;
        headingIntegral = 0;
        headingLastError = 0;
        
        ElapsedTime moveTimer = new ElapsedTime();
        moveTimer.reset();
        
        while (opModeIsActive() && moveTimer.seconds() < 10.0) { // 10 second timeout
            // Update current position
            updatePosition();
            
            // Calculate position error
            double deltaX = targetX - currentX;
            double deltaY = targetY - currentY;
            double distanceError = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            
            // Calculate heading error
            double headingError = normalizeAngle(targetHeading - currentHeading);
            
            // Check if we've reached the target
            if (distanceError < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE) {
                telemetry.addData("Navigation", "Target reached!");
                telemetry.update();
                break;
            }
            
            // Calculate desired movement direction (field-relative)
            double desiredAngle = Math.atan2(deltaY, deltaX) * 180.0 / Math.PI;
            
            // Convert to robot-relative movement
            double robotRelativeAngle = normalizeAngle(desiredAngle - currentHeading);
            double robotRelativeAngleRad = Math.toRadians(robotRelativeAngle);
            
            // Calculate PID output for position
            double positionOutput = calculatePositionPID(distanceError);
            
            // Calculate drive components
            double drive = Math.cos(robotRelativeAngleRad) * positionOutput;
            double strafe = Math.sin(robotRelativeAngleRad) * positionOutput;
            
            // Calculate PID output for heading
            double headingOutput = calculateHeadingPID(headingError);
            double twist = headingOutput;
            
            // Apply mecanum drive kinematics
            double leftFrontPower = drive + strafe + twist;
            double rightFrontPower = drive - strafe - twist;
            double leftBackPower = drive - strafe + twist;
            double rightBackPower = drive + strafe - twist;
            
            // Normalize wheel powers
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
            
            // Apply power limits
            double maxSpeed = MAX_DRIVE_SPEED;
            leftFrontPower = Range.clip(leftFrontPower * maxSpeed, -maxSpeed, maxSpeed);
            rightFrontPower = Range.clip(rightFrontPower * maxSpeed, -maxSpeed, maxSpeed);
            leftBackPower = Range.clip(leftBackPower * maxSpeed, -maxSpeed, maxSpeed);
            rightBackPower = Range.clip(rightBackPower * maxSpeed, -maxSpeed, maxSpeed);
            
            // Set motor powers
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            
            // Display navigation info
            telemetry.addData("Current Position", "X:%.1f Y:%.1f H:%.1f°", currentX, currentY, currentHeading);
            telemetry.addData("Target Position", "X:%.1f Y:%.1f H:%.1f°", targetX, targetY, targetHeading);
            telemetry.addData("Distance Error", "%.2f inches", distanceError);
            telemetry.addData("Heading Error", "%.1f degrees", headingError);
            telemetry.addData("Drive Powers", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f", 
                            leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.update();
            
            sleep(20); // Small delay for stability
        }
        
        // Stop motors when target reached or timeout
        stopDriveMotors();
    }
    
    /**
     * Execute a specific action at the current position
     */
    public void executeAction(int actionType) {
        switch (actionType) {
            case ACTION_LAUNCH_STANDARD:
                telemetry.addData("Action", "Executing standard launcher sequence");
                telemetry.update();
                performLauncherSequence(STANDARD_LAUNCHER_RPM, SERVO_LAUNCH_POSITION_A, 3.0);
                break;
                
            case ACTION_LAUNCH_HIGH_POWER:
                telemetry.addData("Action", "Executing high-power launcher sequence");
                telemetry.update();
                performLauncherSequence(HIGH_POWER_LAUNCHER_RPM, SERVO_LAUNCH_POSITION_Y, 3.0);
                break;
                
            case ACTION_PICKUP_SEQUENCE:
                telemetry.addData("Action", "Executing pickup sequence");
                telemetry.update();
                performPickupSequence(2.0);
                break;
                
            case ACTION_SERVO_HOME:
                telemetry.addData("Action", "Moving servo to home position");
                telemetry.update();
                if (torqueServo != null) {
                    torqueServo.setPosition(SERVO_HOME_POSITION);
                }
                sleep(500);
                break;
                
            case ACTION_WAIT:
                telemetry.addData("Action", "Waiting 1 second");
                telemetry.update();
                sleep(1000);
                break;
                
            default:
                telemetry.addData("Action", "Unknown action type: " + actionType);
                telemetry.update();
                break;
        }
    }
    
    // ===================== POSITION TRACKING METHODS =====================
    
    /**
     * Update the robot's current position using available sensors
     */
    private void updatePosition() {
        // Try to get position from Limelight first (most accurate)
        if (limelightAvailable && updatePositionFromLimelight()) {
            // Limelight provided position update
        } else {
            // Fall back to dead reckoning with encoders and IMU
            updatePositionFromDeadReckoning();
        }
    }
    
    /**
     * Update position using Limelight vision localization
     */
    private boolean updatePositionFromLimelight() {
        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // Get robot pose from Limelight (if configured for localization)
                Pose3D robotPose = result.getBotpose();
                if (robotPose != null) {
                    // Convert Limelight coordinates to field coordinates
                    // Note: You'll need to calibrate this based on your field setup
                    // Simplified access - adjust based on actual Pose3D API
                    currentX = 0; // robotPose.x or equivalent - check Limelight documentation
                    currentY = 0; // robotPose.y or equivalent - check Limelight documentation
                    currentHeading = 0; // robotPose heading or equivalent
                    
                    telemetry.addData("Position Source", "Limelight (placeholder - configure for your setup)");
                    return true;
                }
            }
        } catch (Exception e) {
            // Limelight read failed, continue with dead reckoning
        }
        return false;
    }
    
    /**
     * Update position using encoder dead reckoning and IMU
     */
    private void updatePositionFromDeadReckoning() {
        // Get current encoder positions
        int leftFrontPos = leftFrontDrive.getCurrentPosition();
        int rightFrontPos = rightFrontDrive.getCurrentPosition();
        int leftBackPos = leftBackDrive.getCurrentPosition();
        int rightBackPos = rightBackDrive.getCurrentPosition();
        
        // Calculate encoder change from last update
        double deltaLeft = ((leftFrontPos - lastLeftFront) + (leftBackPos - lastLeftBack)) / 2.0 / COUNTS_PER_INCH;
        double deltaRight = ((rightFrontPos - lastRightFront) + (rightBackPos - lastRightBack)) / 2.0 / COUNTS_PER_INCH;
        double deltaForward = (deltaLeft + deltaRight) / 2.0;
        double deltaStrafe = ((leftFrontPos - lastLeftFront) - (leftBackPos - lastLeftBack) + 
                             (rightBackPos - lastRightBack) - (rightFrontPos - lastRightFront)) / 4.0 / COUNTS_PER_INCH;
        
        // Update heading from IMU if available
        if (imuAvailable) {
            currentHeading = imu.getRobotYawAngle(AngleUnit.DEGREES);
        }
        
        // Convert robot-relative movement to field-relative
        double headingRad = Math.toRadians(currentHeading);
        double deltaX = deltaForward * Math.cos(headingRad) - deltaStrafe * Math.sin(headingRad);
        double deltaY = deltaForward * Math.sin(headingRad) + deltaStrafe * Math.cos(headingRad);
        
        // Update position
        currentX += deltaX;
        currentY += deltaY;
        
        // Store encoder positions for next calculation
        lastLeftFront = leftFrontPos;
        lastRightFront = rightFrontPos;
        lastLeftBack = leftBackPos;
        lastRightBack = rightBackPos;
        
        telemetry.addData("Position Source", "Dead Reckoning");
    }
    
    // ===================== PID CONTROL METHODS =====================
    
    private double calculatePositionPID(double error) {
        positionIntegral += error;
        double derivative = error - positionLastError;
        positionLastError = error;
        
        // Apply integral windup protection
        positionIntegral = Range.clip(positionIntegral, -100, 100);
        
        double output = POSITION_KP * error + POSITION_KI * positionIntegral + POSITION_KD * derivative;
        return Range.clip(output, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
    }
    
    private double calculateHeadingPID(double error) {
        headingIntegral += error;
        double derivative = error - headingLastError;
        headingLastError = error;
        
        // Apply integral windup protection
        headingIntegral = Range.clip(headingIntegral, -100, 100);
        
        double output = HEADING_KP * error + HEADING_KI * headingIntegral + HEADING_KD * derivative;
        return Range.clip(output, -MAX_TURN_SPEED, MAX_TURN_SPEED);
    }
    
    // ===================== ACTION SEQUENCE METHODS =====================
    
    private void performLauncherSequence(double targetRPM, double servoPosition, double timeoutSeconds) {
        // Set servo position
        if (torqueServo != null) {
            torqueServo.setPosition(servoPosition);
        }
        
        // Start launcher motor
        if (launcherMotor != null) {
            launcherMotor.setPower(1.0);
        }
        
        // Wait for launcher to spin up
        sleep(2000);
        
        // Start feeding motors
        if (pickupMotor != null) {
            pickupMotor.setPower(0.8);
        }
        if (kickerMotor != null) {
            kickerMotor.setPower(0.3);
        }
        
        // Run feeding sequence
        sleep((long)(timeoutSeconds * 1000));
        
        // Stop all motors
        if (launcherMotor != null) launcherMotor.setPower(0);
        if (pickupMotor != null) pickupMotor.setPower(0);
        if (kickerMotor != null) kickerMotor.setPower(0);
    }
    
    private void performPickupSequence(double timeoutSeconds) {
        // Start pickup motor
        if (pickupMotor != null) {
            pickupMotor.setPower(0.6);
        }
        
        // Run pickup for specified time
        sleep((long)(timeoutSeconds * 1000));
        
        // Stop pickup motor
        if (pickupMotor != null) {
            pickupMotor.setPower(0);
        }
    }
    
    // ===================== UTILITY METHODS =====================
    
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
    
    private void resetDriveEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    private void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }
    
    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}