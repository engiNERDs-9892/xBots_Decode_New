package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;

/**
 * PID Motor Test OpMode - Advanced Motor Testing + Color Sensor
 * 
 * This OpMode is designed specifically for testing and tuning the three PID motors:
 * - Launcher Motor (A button - 4500 RPM)
 * - Pickup Motor (Y button - 100 RPM)
 * - Kicker Motor (X button - 150 RPM)
 * 
 * Additional features:
 * - REV Robotics Color Sensor V3 with LED control
 * 
 * Use this OpMode to:
 * - Test each motor individually
 * - Tune PID constants
 * - Verify RPM accuracy
 * - Debug motor performance
 * - Test color detection
 * 
 * Controls:
 * A = Launcher Motor (4500 RPM)
 * Y = Pickup Motor (100 RPM)
 * X = Kicker Motor (150 RPM)
 * B = Toggle Color Sensor LED
 * BACK = Emergency stop all motors
 */
@TeleOp(name="PID Motor Test", group="Testing")
public class PIDMotorTest extends LinearOpMode {

    // ===================== HARDWARE DECLARATIONS =====================
    
    private ElapsedTime runtime = new ElapsedTime();
    
    // REV Robotics Color Sensor V3
    private NormalizedColorSensor colorSensor = null;
    private boolean colorSensorLightEnabled = true;
    private boolean lastLightToggleButton = false;
    
    // ===================== PID MOTOR CONTROL SYSTEMS =====================
    
    // Launcher Motor (5203 series 6000rpm) - A button control
    private DcMotor launcherMotor = null;
    private double launcherTargetRPM = 4500.0;
    private double launcherKP = 0.01;
    private double launcherKI = 0.001;
    private double launcherKD = 0.0001;
    private double launcherIntegral = 0;
    private double launcherLastError = 0;
    private ElapsedTime launcherTimer = new ElapsedTime();
    private boolean launcherActive = false;
    private int launcherLastPosition = 0;
    private double launcherLastTime = 0;
    
    // Pickup Motor (5203 series 6000rpm) - Y button control
    private DcMotor pickupMotor = null;
    private double pickupTargetRPM = 100.0;
    private double pickupKP = 0.02;           // Higher gain for lower speed
    private double pickupKI = 0.005;
    private double pickupKD = 0.0001;
    private double pickupIntegral = 0;
    private double pickupLastError = 0;
    private ElapsedTime pickupTimer = new ElapsedTime();
    private boolean pickupActive = false;
    private int pickupLastPosition = 0;
    private double pickupLastTime = 0;
    
    // Kicker Motor (5203 series 312rpm) - X button control
    private DcMotor kickerMotor = null;
    private double kickerTargetRPM = 150.0;
    private double kickerKP = 0.05;        // Higher gain for precision motor
    private double kickerKI = 0.01;
    private double kickerKD = 0.0005;
    private double kickerIntegral = 0;
    private double kickerLastError = 0;
    private ElapsedTime kickerTimer = new ElapsedTime();
    private boolean kickerActive = false;
    private int kickerLastPosition = 0;
    private double kickerLastTime = 0;
    
    // Emergency stop tracking
    private boolean emergencyStop = false;
    
    @Override
    public void runOpMode() {
        
        // ===================== INITIALIZATION =====================
        
        // Initialize REV Robotics Color Sensor V3
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
            
            // Enable the LED (if available) for better color detection
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }
            
            telemetry.addData("Color Sensor", "✅ Connected - REV Color Sensor V3");
        } catch (Exception e) {
            colorSensor = null;
            telemetry.addData("Color Sensor", "❌ Not found - Color detection disabled");
        }
        
        // Initialize PID Motors
        
        // Launcher Motor (5203 series 6000rpm) - A button control
        try {
            launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor");
            launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcherTimer.reset();
            telemetry.addData("Launcher Motor", "✅ Connected - 4500 RPM ready");
        } catch (Exception e) {
            launcherMotor = null;
            telemetry.addData("Launcher Motor", "❌ Not found - Feature disabled");
        }
        
        // Pickup Motor (5203 series 6000rpm) - Y button control
        try {
            pickupMotor = hardwareMap.get(DcMotor.class, "pickup_motor");
            pickupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pickupMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pickupMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            pickupTimer.reset();
            telemetry.addData("Pickup Motor", "✅ Connected - 100 RPM ready");
        } catch (Exception e) {
            pickupMotor = null;
            telemetry.addData("Pickup Motor", "❌ Not found - Feature disabled");
        }
        
        // Kicker Motor (5203 series 312rpm) - X button control
        try {
            kickerMotor = hardwareMap.get(DcMotor.class, "kicker_motor");
            kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            kickerTimer.reset();
            telemetry.addData("Kicker Motor", "✅ Connected - 150 RPM ready");
        } catch (Exception e) {
            kickerMotor = null;
            telemetry.addData("Kicker Motor", "❌ Not found - Feature disabled");
        }
        
        // Display initialization status
        telemetry.addData("=== PID MOTOR TEST ===", "");
        telemetry.addData("Status", "PID Motor Testing Ready");
        telemetry.addData("Purpose", "Individual motor testing and tuning");
        telemetry.addData("", "");
        telemetry.addData("Controls", "BACK=STOP ALL | No button motors");
        telemetry.addData("Targets", "Launcher:4500 | Pickup:100 | Kicker:150 RPM");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // ===================== MAIN CONTROL LOOP =====================
        
        while (opModeIsActive()) {
            
            // ================== EMERGENCY STOP (BACK BUTTON) ==================
            
            if (gamepad1.back) {
                emergencyStop = true;
                // Stop all motors immediately
                if (launcherMotor != null) {
                    launcherMotor.setPower(0);
                    launcherActive = false;
                    launcherIntegral = 0;
                }
                if (pickupMotor != null) {
                    pickupMotor.setPower(0);
                    pickupActive = false;
                    pickupIntegral = 0;
                }
                if (kickerMotor != null) {
                    kickerMotor.setPower(0);
                    kickerActive = false;
                    kickerIntegral = 0;
                }
            } else {
                emergencyStop = false;
            }
            
            // Only run additional controls if not in emergency stop
            if (!emergencyStop) {
                
                // TODO: Add future motor controls here
                // All button controls (A, B, X, Y) have been removed per user request
                
                // Example structure for future features:
                /*
                if (gamepad1.guide) {
                    // Special function toggle
                }
                */
            }
                
                // ================== LAUNCHER MOTOR PID CONTROL ==================
                
                boolean currentAButton = gamepad1.a;
                if (launcherMotor != null) {
                    if (currentAButton) {
                        if (!launcherActive) {
                            launcherActive = true;
                            launcherIntegral = 0;
                            launcherTimer.reset();
                            launcherLastPosition = launcherMotor.getCurrentPosition();
                            launcherLastTime = launcherTimer.seconds();
                        }
                        
                        double currentTime = launcherTimer.seconds();
                        int currentPosition = launcherMotor.getCurrentPosition();
                        double deltaPosition = currentPosition - launcherLastPosition;
                        double deltaTime = currentTime - launcherLastTime;
                        
                        double currentRPM = 0;
                        if (deltaTime > 0) {
                            double revolutionsPerSecond = deltaPosition / 1440.0 / deltaTime;
                            currentRPM = revolutionsPerSecond * 60.0;
                        }
                        
                        double error = launcherTargetRPM - currentRPM;
                        launcherIntegral += error * deltaTime;
                        double derivative = (deltaTime > 0) ? (error - launcherLastError) / deltaTime : 0;
                        
                        double pidOutput = launcherKP * error + launcherKI * launcherIntegral + launcherKD * derivative;
                        double motorPower = Range.clip(pidOutput / 1000.0, -1.0, 1.0);
                        launcherMotor.setPower(motorPower);
                        
                        launcherLastError = error;
                        launcherLastPosition = currentPosition;
                        launcherLastTime = currentTime;
                    } else {
                        if (launcherActive) {
                            launcherActive = false;
                            launcherMotor.setPower(0);
                            launcherIntegral = 0;
                        }
                    }
                }
                
                // ================== PICKUP MOTOR PID CONTROL ==================
                
                boolean currentYButton = gamepad1.y;
                if (pickupMotor != null) {
                    if (currentYButton) {
                        if (!pickupActive) {
                            pickupActive = true;
                            pickupIntegral = 0;
                            pickupTimer.reset();
                            pickupLastPosition = pickupMotor.getCurrentPosition();
                            pickupLastTime = pickupTimer.seconds();
                        }
                        
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
                    } else {
                        if (pickupActive) {
                            pickupActive = false;
                            pickupMotor.setPower(0);
                            pickupIntegral = 0;
                        }
                    }
                }
                
                // ================== KICKER MOTOR PID CONTROL ==================
                
                boolean currentXButton = gamepad1.x;
                if (kickerMotor != null) {
                    if (currentXButton) {
                        if (!kickerActive) {
                            kickerActive = true;
                            kickerIntegral = 0;
                            kickerTimer.reset();
                            kickerLastPosition = kickerMotor.getCurrentPosition();
                            kickerLastTime = kickerTimer.seconds();
                        }
                        
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
                    } else {
                        if (kickerActive) {
                            kickerActive = false;
                            kickerMotor.setPower(0);
                            kickerIntegral = 0;
                        }
                    }
                }
            }
            
            // ================== TELEMETRY DISPLAY ==================
            
            telemetry.addData("=== PID MOTOR TEST ===", "");
            telemetry.addData("Status", emergencyStop ? "🛑 EMERGENCY STOP ACTIVE" : "✅ Running");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("", "");
            
            // Color Sensor telemetry
            if (colorSensor != null) {
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                
                telemetry.addData("=== COLOR SENSOR ===", "");
                telemetry.addData("Red", "%.3f", colors.red);
                telemetry.addData("Green", "%.3f", colors.green);
                telemetry.addData("Blue", "%.3f", colors.blue);
                telemetry.addData("Alpha", "%.3f", colors.alpha);
                
                // Basic color detection
                String detectedColor = "Unknown";
                if (colors.red > colors.green && colors.red > colors.blue) {
                    if (colors.red > 0.3) detectedColor = "🔴 Red";
                } else if (colors.green > colors.red && colors.green > colors.blue) {
                    if (colors.green > 0.3) detectedColor = "🟢 Green";
                } else if (colors.blue > colors.red && colors.blue > colors.green) {
                    if (colors.blue > 0.3) detectedColor = "🔵 Blue";
                } else if (colors.red + colors.green + colors.blue < 0.3) {
                    detectedColor = "⚫ Black";
                } else if (colors.red > 0.7 && colors.green > 0.7 && colors.blue > 0.7) {
                    detectedColor = "⚪ White";
                }
                
                telemetry.addData("Detected Color", detectedColor);
                telemetry.addData("LED Light", colorSensorLightEnabled ? "💡 ON" : "🔦 OFF");
                telemetry.addData("", "");
            }
            
            // Individual motor status
            if (launcherMotor != null || pickupMotor != null || kickerMotor != null) {
                telemetry.addData("=== PID MOTORS ===", "");
                
                // Launcher Motor (A button)
                if (launcherMotor != null) {
                    double currentTime = launcherTimer.seconds();
                    int currentPosition = launcherMotor.getCurrentPosition();
                    double deltaPosition = currentPosition - launcherLastPosition;
                    double deltaTime = currentTime - launcherLastTime;
                    double currentRPM = 0;
                    if (deltaTime > 0) {
                        double revolutionsPerSecond = deltaPosition / 1440.0 / deltaTime;
                        currentRPM = revolutionsPerSecond * 60.0;
                    }
                    
                    String status = launcherActive ? "🟢 ACTIVE" : "⭕ STOPPED";
                    telemetry.addData("🚀 Launcher (A)", "%s", status);
                    telemetry.addData("   Target RPM", "%.0f", launcherTargetRPM);
                    telemetry.addData("   Current RPM", "%.0f", Math.abs(currentRPM));
                    telemetry.addData("   Power", "%.3f", launcherMotor.getPower());
                    telemetry.addData("   Error", "%.1f RPM", launcherTargetRPM - Math.abs(currentRPM));
                }
                
                // Pickup Motor (Y button)
                if (pickupMotor != null) {
                    double currentTime = pickupTimer.seconds();
                    int currentPosition = pickupMotor.getCurrentPosition();
                    double deltaPosition = currentPosition - pickupLastPosition;
                    double deltaTime = currentTime - pickupLastTime;
                    double currentRPM = 0;
                    if (deltaTime > 0) {
                        double revolutionsPerSecond = deltaPosition / 1440.0 / deltaTime;
                        currentRPM = revolutionsPerSecond * 60.0;
                    }
                    
                    String status = pickupActive ? "🟢 ACTIVE" : "⭕ STOPPED";
                    telemetry.addData("🔄 Pickup (Y)", "%s", status);
                    telemetry.addData("   Target RPM", "%.0f", pickupTargetRPM);
                    telemetry.addData("   Current RPM", "%.0f", Math.abs(currentRPM));
                    telemetry.addData("   Power", "%.3f", pickupMotor.getPower());
                    telemetry.addData("   Error", "%.1f RPM", pickupTargetRPM - Math.abs(currentRPM));
                }
                
                // Kicker Motor (X button)
                if (kickerMotor != null) {
                    double currentTime = kickerTimer.seconds();
                    int currentPosition = kickerMotor.getCurrentPosition();
                    double deltaPosition = currentPosition - kickerLastPosition;
                    double deltaTime = currentTime - kickerLastTime;
                    double currentRPM = 0;
                    if (deltaTime > 0) {
                        double revolutionsPerSecond = deltaPosition / 1440.0 / deltaTime;
                        currentRPM = revolutionsPerSecond * 60.0;
                    }
                    
                    String status = kickerActive ? "🟢 ACTIVE" : "⭕ STOPPED";
                    telemetry.addData("⚽ Kicker (X)", "%s", status);
                    telemetry.addData("   Target RPM", "%.0f", kickerTargetRPM);
                    telemetry.addData("   Current RPM", "%.0f", Math.abs(currentRPM));
                    telemetry.addData("   Power", "%.3f", kickerMotor.getPower());
                    telemetry.addData("   Error", "%.1f RPM", kickerTargetRPM - Math.abs(currentRPM));
                }
                
                telemetry.addData("", "");
                telemetry.addData("=== CONTROLS ===", "");
                telemetry.addData("🎮 A Button", "Launcher Motor (4500 RPM)");
                telemetry.addData("🎮 Y Button", "Pickup Motor (100 RPM)");
                telemetry.addData("🎮 X Button", "Kicker Motor (150 RPM)");
                telemetry.addData("� B Button", "Toggle Color Sensor LED");
                telemetry.addData("�🛑 BACK Button", "EMERGENCY STOP ALL");
                
                telemetry.addData("", "");
                telemetry.addData("=== PID CONSTANTS ===", "");
                telemetry.addData("Launcher", "kP=%.3f, kI=%.3f, kD=%.4f", launcherKP, launcherKI, launcherKD);
                telemetry.addData("Pickup", "kP=%.3f, kI=%.3f, kD=%.4f", pickupKP, pickupKI, pickupKD);
                telemetry.addData("Kicker", "kP=%.3f, kI=%.3f, kD=%.4f", kickerKP, kickerKI, kickerKD);
            }
            
            telemetry.update();
        }
    }
    
    // ===================== HELPER METHODS =====================
    
    /**
     * Immediately stop all motors (called on OpMode stop)
     */
    @Override
    public void stop() {
        if (launcherMotor != null) launcherMotor.setPower(0);
        if (pickupMotor != null) pickupMotor.setPower(0);
        if (kickerMotor != null) kickerMotor.setPower(0);
        super.stop();
    }
}