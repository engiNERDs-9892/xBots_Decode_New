package org.firstinspires.ftc.teamcode.robot;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TurtleRobot {
    private static final String TAG = "TurtleRobot";

    public OpMode myOpMode;
    public static Follower follower;
    public PIDController controller, controllerTurret;
    public static double p = 0.2, i = 0.05, d = 0;
    public static double pT = 0.12, iT = 0, dT = 0;
    public static double f = 0.0265;
    public static double target = 0;
    public static double vel = 0;
    public static double alpha = 0.6;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    public DcMotorEx shooterb, shootert, intake, turret;
    public Servo hood;
    public VoltageSensor volt;
    public static double tangle = 40;
    public static double theta = 0;
    public static double shooterX = 135;
    public static double shooterY = 135;
    public Servo latch;
    public static double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;

    public TurtleRobot(OpMode opmode) {
        myOpMode = opmode;
        RobotLog.ii(TAG, "TurtleRobot constructor called with OpMode: %s", opmode.getClass().getSimpleName());
        RobotLog.ii(TAG, "TICKS_PER_DEGREES calculated: %.4f", TICKS_PER_DEGREES);
        RobotLog.ii(TAG, "PID Constants - Shooter: p=%.3f, i=%.3f, d=%.3f, f=%.4f", p, i, d, f);
        RobotLog.ii(TAG, "PID Constants - Turret: p=%.3f, i=%.3f, d=%.3f", pT, iT, dT);
    }

    public void init(HardwareMap hardwareMap) {
        RobotLog.ii(TAG, "=== TurtleRobot initialization started ===");
        long startTime = System.currentTimeMillis();

        try {
            // Initialize PID Controllers
            RobotLog.ii(TAG, "Initializing PID controllers...");
            controller = new PIDController(p, i, d);
            controllerTurret = new PIDController(pT, iT, dT);
            RobotLog.ii(TAG, "PID controllers initialized successfully");

            // Initialize Motors with error checking
            RobotLog.ii(TAG, "Initializing motors...");

            try {
                shooterb = hardwareMap.get(DcMotorEx.class, "sb");
                RobotLog.ii(TAG, "Shooter bottom motor 'sb' initialized successfully");
            } catch (Exception e) {
                RobotLog.ee(TAG, "Failed to initialize shooter bottom motor 'sb': %s", e.getMessage());
                throw e;
            }

            try {
                turret = hardwareMap.get(DcMotorEx.class, "turret");
                int initialPosition = turret.getCurrentPosition();
                RobotLog.ii(TAG, "Turret motor initialized - Initial position: %d ticks", initialPosition);

                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RobotLog.ii(TAG, "Turret encoder reset");

                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RobotLog.ii(TAG, "Turret set to RUN_WITHOUT_ENCODER mode");

                // Verify encoder reset
                int resetPosition = turret.getCurrentPosition();
                RobotLog.ii(TAG, "Turret position after reset: %d ticks", resetPosition);
            } catch (Exception e) {
                RobotLog.ee(TAG, "Failed to initialize turret motor: %s", e.getMessage());
                throw e;
            }

            try {
                shootert = hardwareMap.get(DcMotorEx.class, "st");
                RobotLog.ii(TAG, "Shooter top motor 'st' initialized successfully");
            } catch (Exception e) {
                RobotLog.ee(TAG, "Failed to initialize shooter top motor 'st': %s", e.getMessage());
                throw e;
            }

            try {
                intake = hardwareMap.get(DcMotorEx.class, "intake");
                RobotLog.ii(TAG, "Intake motor initialized successfully");
            } catch (Exception e) {
                RobotLog.ee(TAG, "Failed to initialize intake motor: %s", e.getMessage());
                throw e;
            }

            // Initialize Servos
            RobotLog.ii(TAG, "Initializing servos...");

            try {
                hood = hardwareMap.get(Servo.class, "hood");
                double hoodPosition = hood.getPosition();
                RobotLog.ii(TAG, "Hood servo initialized - Current position: %.3f", hoodPosition);
            } catch (Exception e) {
                RobotLog.ee(TAG, "Failed to initialize hood servo: %s", e.getMessage());
                throw e;
            }

            try {
                latch = hardwareMap.servo.get("latch");
                double latchPosition = latch.getPosition();
                RobotLog.ii(TAG, "Latch servo initialized - Current position: %.3f", latchPosition);
            } catch (Exception e) {
                RobotLog.ee(TAG, "Failed to initialize latch servo: %s", e.getMessage());
                throw e;
            }

            // Initialize Voltage Sensor
            try {
                volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
                double currentVoltage = volt.getVoltage();
                RobotLog.ii(TAG, "Voltage sensor initialized - Current voltage: %.2fV", currentVoltage);

                if (currentVoltage < 11.5) {
                    RobotLog.ww(TAG, "WARNING: Battery voltage is low! Current: %.2fV", currentVoltage);
                } else if (currentVoltage > 13.5) {
                    RobotLog.ww(TAG, "WARNING: Battery voltage is high! Current: %.2fV", currentVoltage);
                }
            } catch (Exception e) {
                RobotLog.ee(TAG, "Failed to initialize voltage sensor: %s", e.getMessage());
                throw e;
            }

            // Initialize Lookup Tables
            RobotLog.ii(TAG, "Building RPM lookup table...");
            RPM.add(20, 350);
            RPM.add(39, 350);
            RPM.add(50, 375);
            RPM.add(60, 400);
            RPM.add(74, 415);
            RPM.add(90, 450);
            RPM.add(180, 450);
            RPM.createLUT();
            RobotLog.ii(TAG, "RPM LUT created with 7 data points (20-180 distance range)");

            RobotLog.ii(TAG, "Building angle lookup table...");
            angle.add(20, 1);
            angle.add(39, 0.7);
            angle.add(50, 0.6);
            angle.add(60, 0.5);
            angle.add(74, 0.4);
            angle.add(90, 0.3);
            angle.add(180, 0.3);
            angle.createLUT();
            RobotLog.ii(TAG, "Angle LUT created with 7 data points (servo range 0.3-1.0)");

            // Test LUT functionality
            double testDistance = 50.0;
            double testRPM = RPM.get(testDistance);
            double testAngle = angle.get(testDistance);
            RobotLog.ii(TAG, "LUT test - Distance: %.1f -> RPM: %.1f, Angle: %.3f", testDistance, testRPM, testAngle);

            long elapsedTime = System.currentTimeMillis() - startTime;
            RobotLog.ii(TAG, "=== TurtleRobot initialization completed successfully in %dms ===", elapsedTime);

        } catch (Exception e) {
            long elapsedTime = System.currentTimeMillis() - startTime;
            RobotLog.ee(TAG, "=== TurtleRobot initialization FAILED after %dms ===", elapsedTime);
            RobotLog.ee(TAG, "Initialization error: %s", e.getMessage());
            RobotLog.ee(TAG, "Stack trace: %s", android.util.Log.getStackTraceString(e));
            throw new RuntimeException("TurtleRobot initialization failed", e);
        }
    }

    /**
     * Logs comprehensive system status for debugging
     */
    public void logSystemStatus() {
        try {
            RobotLog.ii(TAG, "=== System Status Report ===");

            // Voltage monitoring
            if (volt != null) {
                double voltage = volt.getVoltage();
                RobotLog.ii(TAG, "Battery Voltage: %.2fV", voltage);
                if (voltage < 11.5) {
                    RobotLog.ww(TAG, "LOW BATTERY WARNING: %.2fV", voltage);
                }
            }

            // Motor status
            if (shooterb != null) {
                RobotLog.ii(TAG, "Shooter Bottom - Velocity: %.1f ticks/sec, Power: %.3f",
                    shooterb.getVelocity(), shooterb.getPower());
            }
            if (shootert != null) {
                RobotLog.ii(TAG, "Shooter Top - Velocity: %.1f ticks/sec, Power: %.3f",
                    shootert.getVelocity(), shootert.getPower());
            }
            if (turret != null) {
                RobotLog.ii(TAG, "Turret - Position: %d ticks (%.1f degrees), Velocity: %.1f ticks/sec, Power: %.3f",
                    turret.getCurrentPosition(),
                    turret.getCurrentPosition() / TICKS_PER_DEGREES,
                    turret.getVelocity(),
                    turret.getPower());
            }
            if (intake != null) {
                RobotLog.ii(TAG, "Intake - Velocity: %.1f ticks/sec, Power: %.3f",
                    intake.getVelocity(), intake.getPower());
            }

            // Servo positions
            if (hood != null) {
                RobotLog.ii(TAG, "Hood Servo Position: %.3f", hood.getPosition());
            }
            if (latch != null) {
                RobotLog.ii(TAG, "Latch Servo Position: %.3f", latch.getPosition());
            }

            RobotLog.ii(TAG, "=== End Status Report ===");

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error in logSystemStatus: %s", e.getMessage());
        }
    }

    /**
     * Logs PID controller performance for shooter
     */
    public void logShooterPID(double currentVelocity, double targetRPM) {
        if (controller != null) {
            double error = targetRPM - currentVelocity;
            RobotLog.ii(TAG, "Shooter PID - Target: %.1f RPM, Current: %.1f RPM, Error: %.1f, Output: %.4f",
                targetRPM, currentVelocity, error, controller.calculate(currentVelocity, targetRPM));
        }
    }

    /**
     * Logs turret PID controller performance
     */
    public void logTurretPID(double currentPosition, double targetPosition) {
        if (controllerTurret != null && turret != null) {
            double currentDegrees = currentPosition / TICKS_PER_DEGREES;
            double targetDegrees = targetPosition / TICKS_PER_DEGREES;
            double error = targetPosition - currentPosition;
            RobotLog.ii(TAG, "Turret PID - Target: %.1f° (%d ticks), Current: %.1f° (%d ticks), Error: %.1f ticks, Output: %.4f",
                targetDegrees, (int)targetPosition, currentDegrees, (int)currentPosition, error,
                controllerTurret.calculate(currentPosition, targetPosition));
        }
    }

    /**
     * Logs shooting calculations and lookup table results
     */
    public void logShootingCalculations(double distance, double calculatedRPM, double calculatedAngle) {
        RobotLog.ii(TAG, "Shooting Calc - Distance: %.1f, RPM: %.1f, Hood Angle: %.3f",
            distance, calculatedRPM, calculatedAngle);

        // Log interpolation boundaries
        if (RPM != null && angle != null) {
            if (distance < 20 || distance > 180) {
                RobotLog.ww(TAG, "WARNING: Distance %.1f is outside LUT range [20-180]", distance);
            }
        }
    }

    /**
     * Logs turret positioning and angle calculations
     */
    public void logTurretPositioning(double currentX, double currentY, double targetX, double targetY) {
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));

        RobotLog.ii(TAG, "Turret Positioning - Robot: (%.1f, %.1f), Target: (%.1f, %.1f)",
            currentX, currentY, targetX, targetY);
        RobotLog.ii(TAG, "Turret Calc - Distance: %.1f, Angle: %.1f°, Delta: (%.1f, %.1f)",
            distance, angleToTarget, deltaX, deltaY);
    }

    /**
     * Logs motor performance and health metrics
     */
    public void logMotorHealth() {
        RobotLog.ii(TAG, "=== Motor Health Check ===");

        try {
            if (shooterb != null) {
                double current = shooterb.getCurrent(CurrentUnit.AMPS);
                boolean overCurrent = current > 10.0; // 10A threshold
                RobotLog.ii(TAG, "Shooter Bottom - Current: %.2fA %s", current,
                    overCurrent ? "(OVERCURRENT WARNING)" : "(OK)");
                if (overCurrent) {
                    RobotLog.ww(TAG, "Shooter bottom motor drawing excessive current!");
                }
            }

            if (shootert != null) {
                double current = shootert.getCurrent(CurrentUnit.AMPS);
                boolean overCurrent = current > 10.0;
                RobotLog.ii(TAG, "Shooter Top - Current: %.2fA %s", current,
                    overCurrent ? "(OVERCURRENT WARNING)" : "(OK)");
                if (overCurrent) {
                    RobotLog.ww(TAG, "Shooter top motor drawing excessive current!");
                }
            }

            if (turret != null) {
                double current = turret.getCurrent(CurrentUnit.AMPS);
                boolean overCurrent = current > 8.0; // Lower threshold for turret
                RobotLog.ii(TAG, "Turret - Current: %.2fA %s", current,
                    overCurrent ? "(OVERCURRENT WARNING)" : "(OK)");
                if (overCurrent) {
                    RobotLog.ww(TAG, "Turret motor drawing excessive current!");
                }
            }

            if (intake != null) {
                double current = intake.getCurrent(CurrentUnit.AMPS);
                boolean overCurrent = current > 8.0;
                RobotLog.ii(TAG, "Intake - Current: %.2fA %s", current,
                    overCurrent ? "(OVERCURRENT WARNING)" : "(OK)");
                if (overCurrent) {
                    RobotLog.ww(TAG, "Intake motor drawing excessive current!");
                }
            }

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error checking motor health: %s", e.getMessage());
        }

        RobotLog.ii(TAG, "=== End Motor Health Check ===");
    }

    /**
     * Logs error conditions and warnings
     */
    public void logError(String subsystem, String errorMessage) {
        RobotLog.ee(TAG, "ERROR in %s: %s", subsystem, errorMessage);
    }

    /**
     * Logs warning conditions
     */
    public void logWarning(String subsystem, String warningMessage) {
        RobotLog.ww(TAG, "WARNING in %s: %s", subsystem, warningMessage);
    }

    /**
     * Logs performance metrics for optimization
     */
    public void logPerformanceMetrics(long loopTime, double cpuUsage) {
        if (loopTime > 20) { // More than 20ms loop time
            RobotLog.ww(TAG, "PERFORMANCE: Loop time %.1fms (target <20ms)", loopTime);
        } else {
            RobotLog.dd(TAG, "PERFORMANCE: Loop time %.1fms, CPU: %.1f%%", loopTime, cpuUsage);
        }
    }

    /**
     * Emergency shutdown logging
     */
    public void logEmergencyStop(String reason) {
        RobotLog.ee(TAG, "EMERGENCY STOP TRIGGERED: %s", reason);
        RobotLog.ee(TAG, "All motors will be stopped immediately!");

        // Log final system state
        logSystemStatus();
    }
}
