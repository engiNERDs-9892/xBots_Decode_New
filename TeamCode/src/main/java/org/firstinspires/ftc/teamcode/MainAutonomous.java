package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Main Autonomous OpMode
 * 
 * This autonomous OpMode uses the same hardware configuration as MainTeleOpController
 * for consistent robot behavior between TeleOp and Autonomous periods.
 * 
 * Hardware Configuration:
 * - Mecanum drive system (4 motors)
 * - IMU for field-centric navigation
 * - REV Color Sensor V3 for object detection
 * - Three PID motors: Launcher, Pickup, Kicker
 * - goBILDA Torque Servo
 * 
 * Example autonomous sequence included - customize for your competition strategy!
 */
@Autonomous(name="Main Autonomous", group="Competition")
public class MainAutonomous extends LinearOpMode {

    // ===================== HARDWARE DECLARATIONS =====================
    
    // Drive Motors (mecanum drive)
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    
    // IMU for field-centric navigation
    private IMU imu = null;
    
    // REV Robotics Color Sensor V3
    private NormalizedColorSensor colorSensor = null;
    
    // Action Motors (same as TeleOp)
    private DcMotor launcherMotor = null;
    private DcMotor pickupMotor = null;
    private DcMotor kickerMotor = null;
    
    // goBILDA Torque Servo
    private Servo torqueServo = null;
    
    // ===================== AUTONOMOUS VARIABLES =====================
    
    // Timing and control
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stepTimer = new ElapsedTime();
    
    // Drive constants (adjust based on your robot)
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;
    private static final double PRECISION_SPEED = 0.3;
    
    // Encoder-based movement (ticks per inch - calibrate for your robot)
    private static final double COUNTS_PER_INCH = 1120 / (4 * Math.PI); // Example for 4" wheels
    
    // Launcher speeds (matching TeleOp)
    private static final double STANDARD_LAUNCHER_RPM = 4500.0;
    private static final double HIGH_POWER_LAUNCHER_RPM = 4750.0;
    private static final double PICKUP_RPM = 1000.0;
    private static final double KICKER_RPM = 100.0;
    
    // Servo positions (matching TeleOp)
    private static final double SERVO_LAUNCH_POSITION_A = 0.0;  // A button equivalent (0°)
    private static final double SERVO_LAUNCH_POSITION_Y = 1.0;  // Y button equivalent (180°)
    private static final double SERVO_HOME_POSITION = 0.0;     // Default position
    
    @Override
    public void runOpMode() {
        
        // ===================== HARDWARE INITIALIZATION =====================
        
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        
        // Set motor directions (adjust for your robot configuration)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Set drive motors to brake when power is zero
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize IMU
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            telemetry.addData("IMU", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("IMU", "Not found - using robot-centric only");
            imu = null;
        }
        
        // Initialize color sensor
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
            telemetry.addData("Color Sensor", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Color Sensor", "Not found - detection disabled");
            colorSensor = null;
        }
        
        // Initialize action motors
        try {
            launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor");
            launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Launcher Motor", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Launcher Motor", "Not found");
            launcherMotor = null;
        }
        
        try {
            pickupMotor = hardwareMap.get(DcMotor.class, "pickup_motor");
            pickupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pickupMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Pickup Motor", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Pickup Motor", "Not found");
            pickupMotor = null;
        }
        
        try {
            kickerMotor = hardwareMap.get(DcMotor.class, "kicker_motor");
            kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Kicker Motor", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Kicker Motor", "Not found");
            kickerMotor = null;
        }
        
        // Initialize servo
        try {
            torqueServo = hardwareMap.get(Servo.class, "torque_servo");
            torqueServo.setPosition(SERVO_HOME_POSITION);
            telemetry.addData("Torque Servo", "Initialized at home position");
        } catch (Exception e) {
            telemetry.addData("Torque Servo", "Not found");
            torqueServo = null;
        }
        
        // Reset encoders for autonomous movement
        resetDriveEncoders();
        
        telemetry.addData("Status", "Hardware initialization complete");
        telemetry.addData("Ready", "Press PLAY to start autonomous");
        telemetry.update();
        
        // ===================== WAIT FOR START =====================
        
        waitForStart();
        runtime.reset();
        
        if (opModeIsActive()) {
            // ===================== AUTONOMOUS SEQUENCE =====================
            
            telemetry.addData("Status", "Autonomous sequence starting...");
            telemetry.update();
            
            // EXAMPLE AUTONOMOUS SEQUENCE - Customize this for your strategy!
            
            // Step 1: Move forward
            telemetry.addData("Step", "1 - Moving forward");
            telemetry.update();
            driveForward(24, DRIVE_SPEED);  // Move 24 inches forward
            
            // Step 2: Launch sequence (standard launcher)
            telemetry.addData("Step", "2 - Standard launcher sequence");
            telemetry.update();
            performLauncherSequence(STANDARD_LAUNCHER_RPM, SERVO_LAUNCH_POSITION_A, 3.0);
            
            // Step 3: Turn right 90 degrees
            telemetry.addData("Step", "3 - Turning right");
            telemetry.update();
            turnRight(90, TURN_SPEED);
            
            // Step 4: Move forward and look for objects
            telemetry.addData("Step", "4 - Searching for objects");
            telemetry.update();
            driveForwardWithColorDetection(12, PRECISION_SPEED);
            
            // Step 5: High-power launcher sequence
            telemetry.addData("Step", "5 - High-power launcher sequence");
            telemetry.update();
            performLauncherSequence(HIGH_POWER_LAUNCHER_RPM, SERVO_LAUNCH_POSITION_Y, 3.0);
            
            // Step 6: Return servo to home position
            if (torqueServo != null) {
                torqueServo.setPosition(SERVO_HOME_POSITION);
            }
            
            // Step 7: Final positioning
            telemetry.addData("Step", "6 - Final positioning");
            telemetry.update();
            strafeLeft(18, DRIVE_SPEED);    // Strafe 18 inches left
            
            telemetry.addData("Status", "Autonomous sequence complete!");
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.update();
        }
    }
    
    // ===================== AUTONOMOUS MOVEMENT METHODS =====================
    
    /**
     * Drive forward for a specified distance
     */
    private void driveForward(double inches, double speed) {
        int targetTicks = (int)(inches * COUNTS_PER_INCH);
        
        resetDriveEncoders();
        setDriveTarget(targetTicks, targetTicks, targetTicks, targetTicks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        setDrivePower(speed, speed, speed, speed);
        
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Driving", "Forward %.1f inches", inches);
            telemetry.addData("Progress", "LF: %d/%d", leftFrontDrive.getCurrentPosition(), targetTicks);
            telemetry.update();
        }
        
        stopDriveMotors();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Drive backward for a specified distance
     */
    private void driveBackward(double inches, double speed) {
        driveForward(-inches, speed);
    }
    
    /**
     * Strafe left for a specified distance
     */
    private void strafeLeft(double inches, double speed) {
        int targetTicks = (int)(inches * COUNTS_PER_INCH);
        
        resetDriveEncoders();
        setDriveTarget(-targetTicks, targetTicks, targetTicks, -targetTicks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        setDrivePower(speed, speed, speed, speed);
        
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Strafing", "Left %.1f inches", inches);
            telemetry.update();
        }
        
        stopDriveMotors();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Strafe right for a specified distance
     */
    private void strafeRight(double inches, double speed) {
        strafeLeft(-inches, speed);
    }
    
    /**
     * Turn right by a specified angle (approximate)
     */
    private void turnRight(double degrees, double speed) {
        // Approximate turn distance (calibrate for your robot)
        double turnInches = degrees * 0.1; // Adjust this multiplier
        int targetTicks = (int)(turnInches * COUNTS_PER_INCH);
        
        resetDriveEncoders();
        setDriveTarget(targetTicks, -targetTicks, targetTicks, -targetTicks);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        setDrivePower(speed, speed, speed, speed);
        
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Turning", "Right %.1f degrees", degrees);
            telemetry.update();
        }
        
        stopDriveMotors();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Turn left by a specified angle
     */
    private void turnLeft(double degrees, double speed) {
        turnRight(-degrees, speed);
    }
    
    /**
     * Drive forward while monitoring color sensor for object detection
     */
    private void driveForwardWithColorDetection(double maxInches, double speed) {
        int maxTicks = (int)(maxInches * COUNTS_PER_INCH);
        
        resetDriveEncoders();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        setDrivePower(speed, speed, speed, speed);
        
        while (opModeIsActive() && 
               Math.abs(leftFrontDrive.getCurrentPosition()) < maxTicks) {
            
            // Check for color detection
            if (colorSensor != null) {
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                boolean greenDetected = colors.green > colors.red && colors.green > colors.blue && colors.green > 0.3;
                boolean purpleDetected = colors.red > 0.3 && colors.blue > 0.3 && colors.green < 0.2;
                
                if (greenDetected || purpleDetected) {
                    telemetry.addData("Object Detected", greenDetected ? "Green" : "Purple");
                    telemetry.addData("Action", "Stopping drive");
                    telemetry.update();
                    break;
                }
            }
            
            telemetry.addData("Searching", "Looking for objects...");
            telemetry.addData("Distance", "%.1f/%.1f inches", 
                            Math.abs(leftFrontDrive.getCurrentPosition()) / COUNTS_PER_INCH, maxInches);
            telemetry.update();
        }
        
        stopDriveMotors();
    }
    
    // ===================== LAUNCHER SEQUENCE METHODS =====================
    
    /**
     * Perform a complete launcher sequence
     */
    private void performLauncherSequence(double targetRPM, double servoPosition, double timeoutSeconds) {
        telemetry.addData("Launcher", "Starting sequence...");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Servo Position", "%.2f", servoPosition);
        telemetry.update();
        
        // Step 1: Set servo position
        if (torqueServo != null) {
            torqueServo.setPosition(servoPosition);
        }
        
        // Step 2: Start launcher motor
        if (launcherMotor != null) {
            launcherMotor.setPower(1.0); // Full power to reach RPM quickly
        }
        
        // Step 3: Wait for launcher to reach speed (simplified - add PID control for precision)
        stepTimer.reset();
        while (opModeIsActive() && stepTimer.seconds() < 2.0) {
            telemetry.addData("Launcher", "Spinning up... %.1f sec", stepTimer.seconds());
            telemetry.update();
            sleep(50);
        }
        
        // Step 4: Start feeding motors
        if (pickupMotor != null) {
            pickupMotor.setPower(0.8); // Pickup motor at feeding speed
        }
        if (kickerMotor != null) {
            kickerMotor.setPower(0.3); // Kicker motor at feeding speed
        }
        
        // Step 5: Run feeding sequence
        stepTimer.reset();
        while (opModeIsActive() && stepTimer.seconds() < timeoutSeconds) {
            telemetry.addData("Launcher", "Feeding... %.1f/%.1f sec", stepTimer.seconds(), timeoutSeconds);
            telemetry.update();
            sleep(50);
        }
        
        // Step 6: Stop all motors
        if (launcherMotor != null) launcherMotor.setPower(0);
        if (pickupMotor != null) pickupMotor.setPower(0);
        if (kickerMotor != null) kickerMotor.setPower(0);
        
        telemetry.addData("Launcher", "Sequence complete");
        telemetry.update();
        sleep(500); // Brief pause between sequences
    }
    
    // ===================== DRIVE HELPER METHODS =====================
    
    private void resetDriveEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    private void setDriveTarget(int lf, int rf, int lb, int rb) {
        leftFrontDrive.setTargetPosition(lf);
        rightFrontDrive.setTargetPosition(rf);
        leftBackDrive.setTargetPosition(lb);
        rightBackDrive.setTargetPosition(rb);
    }
    
    private void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }
    
    private void setDrivePower(double lf, double rf, double lb, double rb) {
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }
    
    private void stopDriveMotors() {
        setDrivePower(0, 0, 0, 0);
    }
    
    private boolean motorsAreBusy() {
        return leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || 
               leftBackDrive.isBusy() || rightBackDrive.isBusy();
    }
}