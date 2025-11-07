package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Main TeleOp Controller - Streamlined for Drive + Sensors + Motors (No Button Controls)
 * 
 * This is the primary TeleOp OpMode for competition use.
 * Features ready for implementation but no button controls active.
 * 
 * Features:
 * - Field-centric and robot-centric mecanum drive
 * - REV Robotics Color Sensor V3 for detection  
 * - Three PID motors initialized and ready (no controls yet)
 * - Clean, minimal codebase for reliability
 * - Logitech controller support
 * - BACK button resets IMU heading
 * 
 * Ready for Implementation:
 * - Launcher Motor (4500 RPM) - hardware connected, no controls
 * - Pickup Motor (100 RPM) - hardware connected, no controls  
 * - Kicker Motor (150 RPM) - hardware connected, no controls
 * - Color sensor LED control - ready for implementation
 */
@TeleOp(name="Main TeleOp Controller", group="Competition")
public class MainTeleOpController extends LinearOpMode {

    // ===================== HARDWARE DECLARATIONS =====================
    
    // Drive Motors (312 RPM precision motors)
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    
    // IMU for field-centric drive (optional)
    private IMU imu = null;
    
    // REV Robotics Color Sensor V3
    private NormalizedColorSensor colorSensor = null;
    
    private ElapsedTime runtime = new ElapsedTime();
    
    // ===================== DRIVE CONTROL SETTINGS =====================
    
    private double normalSpeed = 0.8;        // Normal driving speed
    private double precisionSpeed = 0.4;     // Precision mode speed
    private double currentMaxSpeed = normalSpeed;
    
    // Acceleration limiting (less critical with 312 RPM motors but still useful)
    private double accelerationLimit = 0.15; // Can be higher with precision motors
    private double lastLeftPower = 0;
    private double lastRightPower = 0;
    private double lastStrafePower = 0;
    
    // Control mode variables
    private boolean fieldRelative = false;
    private boolean lastModeButton = false;
    private boolean lastResetButton = false;
    
    // Pickup motor toggle control
    private boolean pickupToggleActive = false;
    private boolean lastPickupToggleButton = false;
    
    // Pickup system kicker control (independent from launcher kicker)
    private boolean pickupKickerEnabled = true;  // Can kicker run with pickup?
    private boolean pickupKickerActive = false;  // Is kicker currently running with pickup?
    

    
    // ===================== ADDITIONAL FEATURE VARIABLES =====================
    
    // PID Motor Control Systems (ready for future button mapping)
    
    // Launcher Motor (5203 series 6000rpm) - Ready for control implementation
    private DcMotor launcherMotor = null;
    private double launcherTargetRPM = 4500.0;
    private double launcherCurrentRPM = 0.0;
    private double launcherKP = 0.01;
    private double launcherKI = 0.001;
    private double launcherKD = 0.0001;
    private double launcherIntegral = 0;
    private double launcherLastError = 0;
    private ElapsedTime launcherTimer = new ElapsedTime();
    private boolean launcherActive = false;
    private int launcherLastPosition = 0;
    private double launcherLastTime = 0;
    
    // Y Button Launcher Control (alternate launcher with 4750 RPM and servo at 180°)
    private double yLauncherTargetRPM = 4750.0;
    private boolean yLauncherActive = false;
    private ElapsedTime yButtonReleaseTimer = new ElapsedTime();
    private boolean yButtonReleased = false;
    
    // Pickup Motor (5203 series 6000rpm) - Ready for control implementation
    private DcMotor pickupMotor = null;
    private double pickupTargetRPM = 1000.0;
    private double pickupKP = 0.02;           // Higher gain for lower speed
    private double pickupKI = 0.005;
    private double pickupKD = 0.0001;
    private double pickupIntegral = 0;
    private double pickupLastError = 0;
    private ElapsedTime pickupTimer = new ElapsedTime();
    private boolean pickupActive = false;
    private int pickupLastPosition = 0;
    private double pickupLastTime = 0;
    
    // Kicker Motor (5203 series 312rpm) - Ready for control implementation
    private DcMotor kickerMotor = null;
    private double kickerTargetRPM = 100.0;
    private double kickerKP = 0.05;        // Higher gain for precision motor
    private double kickerKI = 0.01;
    private double kickerKD = 0.0005;
    private double kickerIntegral = 0;
    private double kickerLastError = 0;
    private ElapsedTime kickerTimer = new ElapsedTime();
    private boolean kickerActive = false;
    private int kickerLastPosition = 0;
    private double kickerLastTime = 0;
    
    // Color sensor control variables
    private boolean colorSensorLightEnabled = true;
    
    // goBILDA Torque Servo control (ready for future button mapping)
    private Servo torqueServo = null;
    private double servoPosition = 0.5;      // Current position (0.0 to 1.0)
    private double servoMinPosition = 0.0;   // Minimum position
    private double servoMaxPosition = 1.0;   // Maximum position
    private double servoSpeed = 0.02;        // Position change per update (adjustable)
    
    // TODO: Add variables for additional features here
    // Example: private boolean autoMode = false;
    // Example: private ElapsedTime autoTimer = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        
        // ===================== INITIALIZATION =====================
        
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        
        // Set motor directions (adjust based on your robot configuration)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Initialize IMU for field-centric drive (optional)
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            telemetry.addData("IMU Status", "Connected - Field-centric available");
        } catch (Exception e) {
            imu = null;
            fieldRelative = false; // Force robot-centric if no IMU
            telemetry.addData("IMU Status", "Not found - Robot-centric only");
        }
        
        // Initialize REV Robotics Color Sensor V3
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
            
            // Enable the LED (if available) for better color detection
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }
            
            telemetry.addData("Color Sensor", "Connected - REV Color Sensor V3");
        } catch (Exception e) {
            colorSensor = null;
            telemetry.addData("Color Sensor", "Not found - Color detection disabled");
        }
        
        // Initialize goBILDA Torque Servo (ready for future button mapping)
        try {
            torqueServo = hardwareMap.get(Servo.class, "torque_servo");
            torqueServo.setPosition(servoPosition);  // Set to middle position
            telemetry.addData("Torque Servo", "Connected - goBILDA servo ready (no controls yet)");
        } catch (Exception e) {
            torqueServo = null;
            telemetry.addData("Torque Servo", "Not found - Servo control disabled");
        }
        
        // Initialize PID Motors (no button controls yet)
        
        // Launcher Motor (5203 series 6000rpm) - Ready for future control
        try {
            launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor");
            launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcherTimer.reset();
            telemetry.addData("Launcher Motor", "Connected - 4500 RPM ready (no controls yet)");
        } catch (Exception e) {
            launcherMotor = null;
            telemetry.addData("Launcher Motor", "Not found - Feature disabled");
        }
        
        // Pickup Motor (5203 series 6000rpm) - Ready for future control
        try {
            pickupMotor = hardwareMap.get(DcMotor.class, "pickup_motor");
            pickupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pickupMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pickupMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            pickupTimer.reset();
            telemetry.addData("Pickup Motor", "Connected - 100 RPM ready (no controls yet)");
        } catch (Exception e) {
            pickupMotor = null;
            telemetry.addData("Pickup Motor", "Not found - Feature disabled");
        }
        
        // Kicker Motor (5203 series 312rpm) - Ready for future control
        try {
            kickerMotor = hardwareMap.get(DcMotor.class, "kicker_motor");
            kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            kickerTimer.reset();
            telemetry.addData("Kicker Motor", "Connected - 150 RPM ready (no controls yet)");
        } catch (Exception e) {
            kickerMotor = null;
            telemetry.addData("Kicker Motor", "Not found - Feature disabled");
        }
        
        // Display initialization status
        telemetry.addData("Status", "Main TeleOp Controller Ready");
        telemetry.addData("Drive Motors", "Mecanum Drive System");
        telemetry.addData("Features", "Field-centric drive, color sensor, PID motors");
        telemetry.addData("Controls", "Left stick=translate, Right stick=rotate");
        telemetry.addData("Modes", "RB=precision, LB=turbo, START=field-centric, BACK=reset heading");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // ===================== MAIN CONTROL LOOP =====================
        
        while (opModeIsActive()) {
            
            // ================== SPEED MODE SELECTION ==================
            
            if (gamepad1.left_bumper) {
                currentMaxSpeed = precisionSpeed;    // Precision mode
            } else {
                currentMaxSpeed = normalSpeed;       // Normal mode
            }
            
            // ================== FIELD RELATIVE TOGGLE ==================
            
            boolean currentModeButton = gamepad1.start;
            if (currentModeButton && !lastModeButton && imu != null) {
                fieldRelative = !fieldRelative;
            }
            lastModeButton = currentModeButton;
            
            // ================== IMU HEADING RESET ==================
            
            boolean currentResetButton = gamepad1.back;
            if (currentResetButton && !lastResetButton && imu != null) {
                imu.resetYaw();
            }
            lastResetButton = currentResetButton;
            
            // ================== DRIVETRAIN CONTROL ==================
            
            // Get raw joystick inputs
            double rawDrive = -gamepad1.left_stick_y;    // Forward/backward
            double rawStrafe = gamepad1.left_stick_x;     // Left/right
            double rawTwist = gamepad1.right_stick_x;     // Rotation
            
            // Apply deadzone
            double drive = Math.abs(rawDrive) > 0.05 ? rawDrive : 0;
            double strafe = Math.abs(rawStrafe) > 0.05 ? rawStrafe : 0;
            double twist = Math.abs(rawTwist) > 0.05 ? rawTwist : 0;
            
            // Apply cubic scaling for better control
            drive = cubicScale(drive);
            strafe = cubicScale(strafe);
            twist = cubicScale(twist) * 0.9;
            
            // ================== FIELD RELATIVE CALCULATION ==================
            
            if (fieldRelative && imu != null) {
                // Get robot heading
                double robotHeading = imu.getRobotYawAngle(AngleUnit.RADIANS);
                
                // Rotate the drive and strafe values based on robot heading
                double rotX = drive * Math.cos(-robotHeading) - strafe * Math.sin(-robotHeading);
                double rotY = drive * Math.sin(-robotHeading) + strafe * Math.cos(-robotHeading);
                
                drive = rotX;
                strafe = rotY;
                // Note: twist (rotation) stays the same in field-centric
            }
            
            // Calculate motor powers for mecanum drive
            double leftFrontPower = drive + strafe + twist;
            double rightFrontPower = drive - strafe - twist;
            double leftBackPower = drive - strafe + twist;
            double rightBackPower = drive + strafe - twist;
            
            // Normalize wheel powers to prevent any from exceeding 1.0
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
            
            // Apply speed scaling
            leftFrontPower *= currentMaxSpeed;
            rightFrontPower *= currentMaxSpeed;
            leftBackPower *= currentMaxSpeed;
            rightBackPower *= currentMaxSpeed;
            
            // Apply acceleration limiting for smooth control
            leftFrontPower = applyAccelLimit(leftFrontPower, lastLeftPower);
            rightFrontPower = applyAccelLimit(rightFrontPower, lastRightPower);
            leftBackPower = applyAccelLimit(leftBackPower, lastStrafePower);
            rightBackPower = applyAccelLimit(rightBackPower, 
                (lastLeftPower + lastRightPower + lastStrafePower) / 3.0);
            
            // Set motor powers directly
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            
            // Store powers for next acceleration limit calculation
            lastLeftPower = (leftFrontPower + leftBackPower) / 2.0;
            lastRightPower = (rightFrontPower + rightBackPower) / 2.0;
            lastStrafePower = (leftFrontPower + rightBackPower - leftBackPower - rightFrontPower) / 4.0;
            

            

            
            // ================== ADDITIONAL FEATURES SECTION ==================
            
            // ================== LAUNCHER CONTROL SECTION ==================
            
            // A Button Launcher (4500 RPM, servo at 0°) vs Y Button Launcher (4750 RPM, servo at 180°)
            boolean aButtonPressed = gamepad1.a;
            boolean yButtonPressed = gamepad1.y;
            
            if (aButtonPressed || yButtonPressed) {
                // Reset pickup kicker availability for both launchers
                pickupKickerEnabled = true;
                
                if (aButtonPressed) {
                    // A Button: 4500 RPM launcher with servo at 0°
                    setLauncherActive(true);
                    yLauncherActive = false;  // Ensure Y launcher is off
                    setServoPosition(0.0);    // Servo to 0 degrees
                    launcherTargetRPM = 4500.0;
                    
                    // Reset Y button timer since A is active
                    yButtonReleased = false;
                    
                } else if (yButtonPressed) {
                    // Y Button: 4750 RPM launcher with servo at 180°
                    setLauncherActive(true);
                    yLauncherActive = true;   // Track that this is Y launcher
                    setServoPosition(1.0);    // Servo to 180 degrees (1.0 = 180°)
                    launcherTargetRPM = 4750.0;
                    
                    // Reset Y button timer since Y is active
                    yButtonReleased = false;
                }
                
                // Check if launcher is within 2% of current target RPM
                double launcherTolerance = launcherTargetRPM * 0.02; // 2% tolerance
                boolean launcherAtSpeed = Math.abs(launcherCurrentRPM - launcherTargetRPM) <= launcherTolerance;
                
                if (launcherAtSpeed) {
                    // Launcher at speed - activate kicker (100 RPM) and intake (1000 RPM)
                    setKickerActive(true);
                    setPickupActive(true);
                } else {
                    // Launcher not at speed yet - immediately shut off kicker and pickup (safety override)
                    setKickerActive(false);
                    setPickupActive(false);
                }
                
            } else {
                // No launcher buttons pressed
                setLauncherActive(false);
                setKickerActive(false);
                setPickupActive(false);
                pickupKickerActive = false;  // Reset pickup kicker state
                
                // Handle Y button release timing for servo return
                if (yLauncherActive && !yButtonReleased) {
                    // Y button was just released, start timer
                    yButtonReleased = true;
                    yButtonReleaseTimer.reset();
                }
                yLauncherActive = false;  // Y launcher no longer active
            }
            
            // Y Button servo return logic - return to 0° after 1 second if Y not pressed again
            if (yButtonReleased && yButtonReleaseTimer.seconds() >= 1.0) {
                if (!gamepad1.y && !gamepad1.a) {  // Only if neither launcher button is pressed
                    setServoPosition(0.0);  // Return servo to 0 degrees
                    yButtonReleased = false;  // Reset flag
                }
            }
            
            // Pickup Motor Toggle - RB button (works independently but launcher overrides)
            if (gamepad1.right_bumper && !lastPickupToggleButton) {
                pickupToggleActive = !pickupToggleActive;
                // Only apply toggle state if no launcher system is active
                if (!gamepad1.a && !gamepad1.y) {
                    if (pickupToggleActive) {
                        setPickupActive(true);
                        // Start pickup kicker if enabled (not blocked by color sensor)
                        if (pickupKickerEnabled) {
                            pickupKickerActive = true;
                            setKickerActive(true);
                        }
                    } else {
                        setPickupActive(false);
                        pickupKickerActive = false;
                        setKickerActive(false);
                    }
                }
                // If any launcher is active (A or Y pressed), toggle state is saved but motor control is overridden
            }
            lastPickupToggleButton = gamepad1.right_bumper;
            
            // Smart pickup kicker control - stop when green or purple object detected
            if (pickupToggleActive && !gamepad1.a && !gamepad1.y && pickupKickerActive) {
                // Check color sensor for green or purple objects
                if (colorSensor != null) {
                    NormalizedRGBA colors = colorSensor.getNormalizedColors();
                    boolean greenDetected = colors.green > colors.red && colors.green > colors.blue && colors.green > 0.3;
                    boolean purpleDetected = colors.red > 0.3 && colors.blue > 0.3 && colors.green < 0.2;
                    
                    if (greenDetected || purpleDetected) {
                        // Object detected - stop kicker and disable until launcher is used
                        pickupKickerActive = false;
                        pickupKickerEnabled = false;
                        setKickerActive(false);
                    }
                }
            }

            /*
            // Future button mappings (currently commented out):
            /*
            // Color Sensor LED - B button toggle
            if (gamepad1.b && !lastLightToggleButton) {
                toggleColorSensorLight();
            }
            lastLightToggleButton = gamepad1.b;
            */
            
            // Update active PID motors
            updateLauncherPID();
            updatePickupPID(); 
            updateKickerPID();
            
            // Example structure for future features:
            /*
            if (gamepad1.guide) {
                // Special function toggle
            }
            
            if (gamepad2.a) {
                // Automated sequence
            }
            */
            
            // ================== TELEMETRY DISPLAY ==================
            
            String speedMode = gamepad1.left_bumper ? "PRECISION" : "NORMAL";
            
            String driveMode = "ROBOT CENTRIC";
            if (imu == null) {
                driveMode = "ROBOT CENTRIC (No IMU)";
            } else if (fieldRelative) {
                driveMode = "FIELD CENTRIC";
            }
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed Mode", speedMode + " (%.1f)", currentMaxSpeed);
            telemetry.addData("Drive Mode", driveMode);
            
            if (imu != null) {
                telemetry.addData("Robot Heading", "%.1f°", 
                                imu.getRobotYawAngle(AngleUnit.DEGREES));
                telemetry.addData("Controls", "START=toggle field/robot, BACK=reset heading");
            }
            
            telemetry.addData("Input", "D:%.2f S:%.2f T:%.2f", drive, strafe, twist);
            telemetry.addData("Motors", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f", 
                            leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            // Color Sensor telemetry
            if (colorSensor != null) {
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                
                telemetry.addData("=== COLOR SENSOR ===", "");
                telemetry.addData("Red", "%.3f", colors.red);
                telemetry.addData("Green", "%.3f", colors.green);
                telemetry.addData("Blue", "%.3f", colors.blue);
                telemetry.addData("Alpha", "%.3f", colors.alpha);
                
                // Enhanced color detection for pickup system
                String detectedColor = "Unknown";
                boolean greenDetected = false;
                boolean purpleDetected = false;
                
                if (colors.red > colors.green && colors.red > colors.blue) {
                    if (colors.red > 0.3) detectedColor = "Red";
                } else if (colors.green > colors.red && colors.green > colors.blue) {
                    if (colors.green > 0.3) {
                        detectedColor = "Green";
                        greenDetected = true;
                    }
                } else if (colors.blue > colors.red && colors.blue > colors.green) {
                    if (colors.blue > 0.3) detectedColor = "Blue";
                } else if (colors.red > 0.3 && colors.blue > 0.3 && colors.green < 0.2) {
                    detectedColor = "Purple";
                    purpleDetected = true;
                } else if (colors.red + colors.green + colors.blue < 0.3) {
                    detectedColor = "Black";
                } else if (colors.red > 0.7 && colors.green > 0.7 && colors.blue > 0.7) {
                    detectedColor = "White";
                }
                
                telemetry.addData("Detected Color", detectedColor);
                if (greenDetected || purpleDetected) {
                    telemetry.addData("Pickup Target", "DETECTED - Kicker stopped");
                }
                telemetry.addData("LED Light", colorSensorLightEnabled ? "ON" : "OFF");
            }
            
            // PID Motors telemetry - Launcher System Active
            if (launcherMotor != null || pickupMotor != null || kickerMotor != null) {
                telemetry.addData("=== LAUNCHER SYSTEM ===", "");
                
                // Show launcher status
                if (launcherMotor != null) {
                    String launcherStatus = launcherActive ? 
                        String.format("Active: %.0f/%.0f RPM", launcherCurrentRPM, launcherTargetRPM) : 
                        "Ready (A button to launch)";
                    telemetry.addData("Launcher Motor", launcherStatus);
                    
                    if (launcherActive) {
                        double tolerance = launcherTargetRPM * 0.02;
                        boolean atSpeed = Math.abs(launcherCurrentRPM - launcherTargetRPM) <= tolerance;
                        telemetry.addData("At Speed (±2%)", atSpeed ? "YES - Feeding Active" : "NO - Spinning Up");
                    }
                }
                
                if (pickupMotor != null) {
                    String pickupStatus;
                    if (pickupActive) {
                        if (launcherActive) {
                            // Launcher system is controlling pickup
                            double tolerance = launcherTargetRPM * 0.02;
                            boolean atSpeed = Math.abs(launcherCurrentRPM - launcherTargetRPM) <= tolerance;
                            pickupStatus = atSpeed ? 
                                "Active: 1000 RPM (Launcher feeding)" : 
                                "Active: 1000 RPM (Should not happen)"; // This shouldn't occur with new logic
                        } else {
                            // Independent toggle operation
                            pickupStatus = "Active: 1000 RPM (Toggle Active)";
                        }
                    } else {
                        if (launcherActive) {
                            // Launcher is active but pickup is off (not at speed yet)
                            pickupStatus = "OFF - Waiting for launcher speed";
                        } else {
                            // No launcher, show toggle state
                            pickupStatus = pickupToggleActive ? 
                                "Toggle ON - Ready" : 
                                "Ready (RB toggle or A launcher)";
                        }
                    }
                    telemetry.addData("Pickup Motor", pickupStatus);
                }
                
                if (kickerMotor != null) {
                    String kickerStatus;
                    if (kickerActive) {
                        if (launcherActive) {
                            kickerStatus = "Active: 100 RPM (Launcher feeding)";
                        } else if (pickupKickerActive) {
                            kickerStatus = "Active: 100 RPM (Pickup assist)";
                        } else {
                            kickerStatus = "Active: 100 RPM";
                        }
                    } else {
                        if (launcherActive) {
                            kickerStatus = "Ready (Launcher not at speed)";
                        } else if (pickupToggleActive && !pickupKickerEnabled) {
                            kickerStatus = "Disabled (Object detected - use A to reset)";
                        } else if (pickupToggleActive && pickupKickerEnabled) {
                            kickerStatus = "Ready (Pickup assist available)";
                        } else {
                            kickerStatus = "Ready (A/Y launcher or RB pickup)";
                        }
                    }
                    telemetry.addData("Kicker Motor", kickerStatus);
                }
                
                telemetry.addData("Controls", "A=Launch(4500)+Servo(0°) | Y=Launch(4750)+Servo(180°) | RB=Smart Pickup");
            }
            
            // Servo telemetry (servo controlled by A/Y launcher systems)
            if (torqueServo != null) {
                telemetry.addData("=== SERVO CONTROL ===", "");
                String servoStatus;
                if (gamepad1.a) {
                    servoStatus = "A LAUNCHER - Position 0° (0.0)";
                } else if (gamepad1.y) {
                    servoStatus = "Y LAUNCHER - Position 180° (1.0)";
                } else if (yButtonReleased) {
                    double timeLeft = 1.0 - yButtonReleaseTimer.seconds();
                    if (timeLeft > 0) {
                        servoStatus = String.format("Returning to 0° in %.1f sec", timeLeft);
                    } else {
                        servoStatus = String.format("Position: %.2f", servoPosition);
                    }
                } else {
                    servoStatus = String.format("Position: %.2f", servoPosition);
                }
                telemetry.addData("Torque Servo", servoStatus);
                telemetry.addData("Servo Range", String.format("%.1f to %.1f", servoMinPosition, servoMaxPosition));
                telemetry.addData("Servo Control", "A=0°(4500RPM) | Y=180°(4750RPM) | Auto-return after 1sec");
            }
            
            telemetry.update();
        }
    }
    
    // ===================== HELPER METHODS =====================
    
    /**
     * Apply cubic scaling for smoother control
     * Provides more precise control at low speeds while maintaining full power at high speeds
     */
    private double cubicScale(double input) {
        return input * input * input * 0.7 + input * 0.3;
    }
    
    /**
     * Apply acceleration limiting for smooth control
     * Less critical with 312 RPM motors but still provides smoother operation
     */
    private double applyAccelLimit(double targetPower, double currentPower) {
        double powerChange = targetPower - currentPower;
        
        if (Math.abs(powerChange) > accelerationLimit) {
            if (powerChange > 0) {
                return currentPower + accelerationLimit;
            } else {
                return currentPower - accelerationLimit;
            }
        }
        
        return targetPower;
    }
    
    // ===================== ADDITIONAL METHODS SECTION =====================
    
    // TODO: Add additional helper methods here
    // This section is reserved for future method implementations such as:
    // - Automated sequences
    // - Complex control algorithms
    // - Competition-specific functions
    
    // Example method structure:
    /*
    private void runAutomatedSequence() {
        // Automated sequence logic
    }
    
    private boolean isTargetDetected() {
        // Vision or sensor-based detection
        return false;
    }
    */
    
    // ===================== PID MOTOR CONTROL METHODS =====================
    
    /**
     * Control the launcher motor with PID to target RPM
     * Call this method to activate/deactivate launcher motor
     */
    private void setLauncherActive(boolean active) {
        if (launcherMotor == null) return;
        
        if (active && !launcherActive) {
            launcherActive = true;
            launcherIntegral = 0;
            launcherTimer.reset();
            launcherLastPosition = launcherMotor.getCurrentPosition();
            launcherLastTime = launcherTimer.seconds();
        } else if (!active && launcherActive) {
            launcherActive = false;
            launcherMotor.setPower(0);
            launcherIntegral = 0;
        }
    }
    
    /**
     * Update launcher motor PID control - call this in main loop when active
     */
    private void updateLauncherPID() {
        if (launcherMotor == null || !launcherActive) {
            launcherCurrentRPM = 0.0; // Reset RPM when inactive
            return;
        }
        
        double currentTime = launcherTimer.seconds();
        int currentPosition = launcherMotor.getCurrentPosition();
        double deltaPosition = currentPosition - launcherLastPosition;
        double deltaTime = currentTime - launcherLastTime;
        
        if (deltaTime > 0) {
            double revolutionsPerSecond = deltaPosition / 1440.0 / deltaTime;
            launcherCurrentRPM = revolutionsPerSecond * 60.0;
        }
        
        double error = launcherTargetRPM - launcherCurrentRPM;
        launcherIntegral += error * deltaTime;
        double derivative = (deltaTime > 0) ? (error - launcherLastError) / deltaTime : 0;
        
        double pidOutput = launcherKP * error + launcherKI * launcherIntegral + launcherKD * derivative;
        double motorPower = Range.clip(pidOutput / 1000.0, -1.0, 1.0);
        launcherMotor.setPower(motorPower);
        
        launcherLastError = error;
        launcherLastPosition = currentPosition;
        launcherLastTime = currentTime;
    }
    
    /**
     * Control the pickup motor with PID to target RPM
     */
    private void setPickupActive(boolean active) {
        if (pickupMotor == null) return;
        
        if (active && !pickupActive) {
            pickupActive = true;
            pickupIntegral = 0;
            pickupTimer.reset();
            pickupLastPosition = pickupMotor.getCurrentPosition();
            pickupLastTime = pickupTimer.seconds();
        } else if (!active && pickupActive) {
            pickupActive = false;
            pickupMotor.setPower(0);
            pickupIntegral = 0;
        }
    }
    
    /**
     * Update pickup motor PID control - call this in main loop when active
     */
    private void updatePickupPID() {
        if (pickupMotor == null || !pickupActive) return;
        
        double currentTime = pickupTimer.seconds();
        int currentPosition = pickupMotor.getCurrentPosition();
        double deltaPosition = currentPosition - pickupLastPosition;
        double deltaTime = currentTime - pickupLastTime;
        
        double currentRPM = 0;
        if (deltaTime > 0) {
            double revolutionsPerSecond = deltaPosition / 1440.0 / deltaTime;
            currentRPM = revolutionsPerSecond * 60.0;
        }
        
        double error = pickupTargetRPM - currentRPM;
        pickupIntegral += error * deltaTime;
        double derivative = (deltaTime > 0) ? (error - pickupLastError) / deltaTime : 0;
        
        double pidOutput = pickupKP * error + pickupKI * pickupIntegral + pickupKD * derivative;
        double motorPower = Range.clip(pidOutput / 1000.0, -1.0, 1.0);
        pickupMotor.setPower(motorPower);
        
        pickupLastError = error;
        pickupLastPosition = currentPosition;
        pickupLastTime = currentTime;
    }
    
    /**
     * Control the kicker motor with PID to target RPM
     */
    private void setKickerActive(boolean active) {
        if (kickerMotor == null) return;
        
        if (active && !kickerActive) {
            kickerActive = true;
            kickerIntegral = 0;
            kickerTimer.reset();
            kickerLastPosition = kickerMotor.getCurrentPosition();
            kickerLastTime = kickerTimer.seconds();
        } else if (!active && kickerActive) {
            kickerActive = false;
            kickerMotor.setPower(0);
            kickerIntegral = 0;
        }
    }
    
    /**
     * Update kicker motor PID control - call this in main loop when active
     */
    private void updateKickerPID() {
        if (kickerMotor == null || !kickerActive) return;
        
        double currentTime = kickerTimer.seconds();
        int currentPosition = kickerMotor.getCurrentPosition();
        double deltaPosition = currentPosition - kickerLastPosition;
        double deltaTime = currentTime - kickerLastTime;
        
        double currentRPM = 0;
        if (deltaTime > 0) {
            double revolutionsPerSecond = deltaPosition / 1440.0 / deltaTime;
            currentRPM = revolutionsPerSecond * 60.0;
        }
        
        double error = kickerTargetRPM - currentRPM;
        kickerIntegral += error * deltaTime;
        double derivative = (deltaTime > 0) ? (error - kickerLastError) / deltaTime : 0;
        
        double pidOutput = kickerKP * error + kickerKI * kickerIntegral + kickerKD * derivative;
        double motorPower = Range.clip(pidOutput / 1000.0, -1.0, 1.0);
        kickerMotor.setPower(motorPower);
        
        kickerLastError = error;
        kickerLastPosition = currentPosition;
        kickerLastTime = currentTime;
    }
    
    /**
     * Toggle the color sensor LED on/off
     */
    private void toggleColorSensorLight() {
        if (colorSensor != null && colorSensor instanceof SwitchableLight) {
            colorSensorLightEnabled = !colorSensorLightEnabled;
            ((SwitchableLight) colorSensor).enableLight(colorSensorLightEnabled);
        }
    }
    
    /**
     * Get current color sensor reading
     */
    private String getDetectedColor() {
        if (colorSensor == null) return "No Sensor";
        
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        
        if (colors.red > colors.green && colors.red > colors.blue) {
            if (colors.red > 0.3) return "Red";
        } else if (colors.green > colors.red && colors.green > colors.blue) {
            if (colors.green > 0.3) return "Green";
        } else if (colors.blue > colors.red && colors.blue > colors.green) {
            if (colors.blue > 0.3) return "Blue";
        } else if (colors.red + colors.green + colors.blue < 0.3) {
            return "Black";
        } else if (colors.red > 0.7 && colors.green > 0.7 && colors.blue > 0.7) {
            return "White";
        }
        return "Unknown";
    }
    
    // ===================== SERVO CONTROL METHODS =====================
    
    /**
     * Set servo to specific position (0.0 to 1.0)
     */
    private void setServoPosition(double position) {
        if (torqueServo == null) return;
        
        position = Range.clip(position, servoMinPosition, servoMaxPosition);
        servoPosition = position;
        torqueServo.setPosition(position);
    }
    
    /**
     * Move servo incrementally (positive = increase position, negative = decrease)
     */
    private void moveServoIncremental(double increment) {
        if (torqueServo == null) return;
        
        double newPosition = servoPosition + increment;
        setServoPosition(newPosition);
    }
    
    /**
     * Move servo to minimum position (0.0)
     */
    private void setServoMin() {
        setServoPosition(servoMinPosition);
    }
    
    /**
     * Move servo to maximum position (1.0)
     */
    private void setServoMax() {
        setServoPosition(servoMaxPosition);
    }
    
    /**
     * Move servo to middle position (0.5)
     */
    private void setServoCenter() {
        setServoPosition(0.5);
    }
    
    /**
     * Get current servo position
     */
    private double getServoPosition() {
        return servoPosition;
    }
    
    /**
     * Set custom servo speed for incremental movement
     */
    private void setServoSpeed(double speed) {
        servoSpeed = Math.abs(speed);  // Ensure positive value
    }
    
    /**
     * Move servo smoothly towards target position
     */
    private void updateServoSmooth(double targetPosition) {
        if (torqueServo == null) return;
        
        targetPosition = Range.clip(targetPosition, servoMinPosition, servoMaxPosition);
        
        if (Math.abs(servoPosition - targetPosition) > 0.01) {  // Dead zone to prevent jitter
            double direction = targetPosition > servoPosition ? 1 : -1;
            moveServoIncremental(direction * servoSpeed);
        }
    }
}