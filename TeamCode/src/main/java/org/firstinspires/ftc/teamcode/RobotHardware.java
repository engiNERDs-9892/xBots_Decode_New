package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Hardware configuration for robot with goBILDA 5203 series motors
 * - Low RPM motors (312 RPM) for precise drivetrain control
 * - High RPM motors (6000+ RPM) for fast mechanisms
 * - Logitech controller support
 */
public class RobotHardware {
    
    /* Public OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor leftBackDrive    = null;
    public DcMotor rightBackDrive   = null;
    
    // High speed motors (6000+ RPM goBILDA motors)
    public DcMotor armMotor         = null;
    public DcMotor intakeMotor      = null;
    public DcMotor liftMotor        = null;
    
    // PID controlled motors
    public DcMotor launcherMotor    = null;  // 5203 series 6000rpm for launcher (4500 RPM target)
    public DcMotor pickupMotor      = null;  // 5203 series 6000rpm for pickup (100 RPM target)
    public DcMotor kickerMotor      = null;  // 5203 series 312rpm for kicker tasks (150 RPM target)
    
    // Servos
    public Servo clawServo          = null;
    public Servo wristServo         = null;
    
    // IMU for field-centric drive (optional)
    public IMU imu                  = null;
    
    /* Local OpMode members. */
    HardwareMap hwMap               = null;
    
    /* Constructor */
    public RobotHardware() {
    }
    
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        
        // Define and Initialize Motors (note: need to use the names in your robot configuration)
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hwMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hwMap.get(DcMotor.class, "right_back_drive");
        
        // High speed motors for mechanisms
        armMotor       = hwMap.get(DcMotor.class, "arm_motor");
        intakeMotor    = hwMap.get(DcMotor.class, "intake_motor");
        liftMotor      = hwMap.get(DcMotor.class, "lift_motor");
        
        // PID controlled motors
        launcherMotor  = hwMap.get(DcMotor.class, "launcher_motor");   // 5203 series 6000rpm
        pickupMotor    = hwMap.get(DcMotor.class, "pickup_motor");     // 5203 series 6000rpm
        kickerMotor    = hwMap.get(DcMotor.class, "kicker_motor");     // 5203 series 312rpm
        
        // Initialize servos
        clawServo      = hwMap.get(Servo.class, "claw_servo");
        wristServo     = hwMap.get(Servo.class, "wrist_servo");
        
        // Initialize IMU (optional - for field-centric drive)
        try {
            imu = hwMap.get(IMU.class, "imu");
            
            // Initialize IMU with hub orientation
            // Adjust these values based on how your Control Hub is mounted:
            // LogoFacingDirection: Which way the REV logo faces
            // UsbFacingDirection: Which way the USB ports face
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,      // REV logo facing up
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD   // USB ports facing forward
            ));
            
            imu.initialize(parameters);
            imu.resetYaw(); // Reset heading to 0
            
        } catch (Exception e) {
            imu = null; // IMU not configured in robot config - field-centric will be disabled
        }
        
        // Set motor directions (adjust these based on your robot's wiring)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // High speed motors - may need direction adjustment based on mounting
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        
        // Set all motors to zero power
        setDrivePower(0, 0, 0, 0);
        armMotor.setPower(0);
        intakeMotor.setPower(0);
        liftMotor.setPower(0);
        
        // Set motor run modes
        // Precision drive motors (312 RPM) - excellent for controlled driving
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // High speed motors - good for fast mechanisms
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // PID motors setup - all need encoders for RPM control
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        pickupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickupMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pickupMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set servo positions to safe starting positions
        clawServo.setPosition(0.5);  // Mid position
        wristServo.setPosition(0.5); // Mid position
    }
    
    /**
     * Set drive motor powers for mecanum drive
     * @param leftFront Left front motor power (-1.0 to 1.0)
     * @param rightFront Right front motor power (-1.0 to 1.0) 
     * @param leftBack Left back motor power (-1.0 to 1.0)
     * @param rightBack Right back motor power (-1.0 to 1.0)
     */
    public void setDrivePower(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }
    
    /**
     * Set tank drive powers (for simplified driving)
     * @param leftPower Left side power (-1.0 to 1.0)
     * @param rightPower Right side power (-1.0 to 1.0)
     */
    public void setTankDrive(double leftPower, double rightPower) {
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
    }
    
    /**
     * Mecanum drive calculation
     * @param drive Forward/backward movement
     * @param strafe Left/right movement  
     * @param twist Rotation
     * @param powerScale Overall power scaling (0.0 to 1.0)
     */
    public void mecanumDrive(double drive, double strafe, double twist, double powerScale) {
        double leftFrontPower = (drive + strafe + twist) * powerScale;
        double rightFrontPower = (drive - strafe - twist) * powerScale;
        double leftBackPower = (drive - strafe + twist) * powerScale;
        double rightBackPower = (drive + strafe - twist) * powerScale;
        
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
    
    /**
     * Control arm with high speed motor (6000+ RPM)
     * @param power Arm power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {
        armMotor.setPower(power);
    }
    
    /**
     * Control intake with high speed motor
     * @param power Intake power (-1.0 to 1.0)
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }
    
    /**
     * Control lift with high speed motor  
     * @param power Lift power (-1.0 to 1.0)
     */
    public void setLiftPower(double power) {
        liftMotor.setPower(power);
    }
    
    /**
     * Set claw servo position
     * @param position Servo position (0.0 to 1.0)
     */
    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }
    
    /**
     * Set wrist servo position
     * @param position Servo position (0.0 to 1.0)
     */
    public void setWristPosition(double position) {
        wristServo.setPosition(position);
    }
    
    /**
     * Get drive motor encoder positions (useful for autonomous)
     */
    public int[] getDriveEncoders() {
        return new int[]{
            leftFrontDrive.getCurrentPosition(),
            rightFrontDrive.getCurrentPosition(),
            leftBackDrive.getCurrentPosition(),
            rightBackDrive.getCurrentPosition()
        };
    }
    
    /**
     * Reset all drive encoders
     */
    public void resetDriveEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Check if IMU is available and working
     */
    public boolean isIMUAvailable() {
        return imu != null;
    }
    
    /**
     * Reset IMU heading to zero
     */
    public void resetIMUHeading() {
        if (imu != null) {
            imu.resetYaw();
        }
    }
    
    /**
     * Get current robot heading in degrees
     */
    public double getHeadingDegrees() {
        if (imu != null) {
            return imu.getRobotYawAngle(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        }
        return 0.0;
    }
    
    /**
     * Get current robot heading in radians
     */
    public double getHeadingRadians() {
        if (imu != null) {
            return imu.getRobotYawAngle(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
        }
        return 0.0;
    }
}