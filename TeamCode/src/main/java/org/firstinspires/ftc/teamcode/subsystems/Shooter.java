package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.util.LoggingConfig;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private static final String TAG = "Shooter";


    private final MotorEx shootert, shooterb, turret;
    private final ServoEx hood;
    private VoltageSensor volt;
    private final Supplier<Pose> poseSupplier;
    private boolean flywheelOn = false;
    private static double vel = 0, target = 0;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    private double shooterX, shooterY;
    private PIDController controllerShooter, controllerTurret;
    public static double p = 0.2, i = 0.05, d = 0;
    public static double pT = 0.12, iT = 0, dT = 0;
    public static double f = 0.0265;
    public static double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;

    // Logging and monitoring
    private ElapsedTime periodicTimer = new ElapsedTime();
    private int periodicCount = 0;
    private long lastHealthCheck = 0;
    private boolean lastFlywheelState = false;

    public Shooter(final HardwareMap hMap, Supplier<Pose> poseSupplier, double shooterX, double shooterY) {
        RobotLog.ii(TAG, "=== Shooter subsystem initialization started ===");
        long startTime = System.currentTimeMillis();

        try {
            this.shooterX = shooterX;
            this.shooterY = shooterY;
            this.poseSupplier = poseSupplier;
            RobotLog.ii(TAG, "Shooter target coordinates: (%.1f, %.1f)", shooterX, shooterY);

            // Initialize motors
            RobotLog.ii(TAG, "Initializing shooter motors...");
            shootert = new MotorEx(hMap, "st");
            RobotLog.ii(TAG, "Shooter top motor initialized");

            shooterb = new MotorEx(hMap, "sb");
            RobotLog.ii(TAG, "Shooter bottom motor initialized");

            turret = new MotorEx(hMap, "turret");
            RobotLog.ii(TAG, "Turret motor initialized");

            hood = new ServoEx(hMap, "latch");
            RobotLog.ii(TAG, "Hood servo initialized");

            volt = hMap.get(VoltageSensor.class, "Control Hub");
            double initialVoltage = volt.getVoltage();
            RobotLog.ii(TAG, "Voltage sensor initialized - Current: %.2fV", initialVoltage);

            // Configure motor modes
            shooterb.setRunMode(MotorEx.RunMode.RawPower);
            shootert.setRunMode(MotorEx.RunMode.RawPower);
            turret.setRunMode(MotorEx.RunMode.RawPower);
            RobotLog.ii(TAG, "All motors set to RawPower mode");

            // Initialize PID controllers
            controllerShooter = new PIDController(p, i, d);
            controllerTurret = new PIDController(pT, iT, dT);
            RobotLog.ii(TAG, "PID controllers initialized - Shooter: p=%.3f, i=%.3f, d=%.3f", p, i, d);
            RobotLog.ii(TAG, "PID controllers initialized - Turret: p=%.3f, i=%.3f, d=%.3f", pT, iT, dT);
            RobotLog.ii(TAG, "Turret TICKS_PER_DEGREES: %.4f", TICKS_PER_DEGREES);

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error during motor initialization: %s", e.getMessage());
            throw e;
        }

        // Initialize lookup tables
        RobotLog.ii(TAG, "Building shooter RPM lookup table...");
        RPM.add(0, 330);
        RPM.add(39, 330);
        RPM.add(50, 355);
        RPM.add(60, 380);
        RPM.add(74, 395);
        RPM.add(90, 430);
        RPM.add(180, 430);
        RPM.createLUT();
        RobotLog.ii(TAG, "RPM LUT created with 7 data points (0-180 distance range)");

        RobotLog.ii(TAG, "Building hood angle lookup table...");
        angle.add(0, 1);
        angle.add(20, 1);
        angle.add(39, 0.7);
        angle.add(50, 0.6);
        angle.add(60, 0.5);
        angle.add(74, 0.4);
        angle.add(90, 0.3);
        angle.add(180, 0.3);
        angle.createLUT();
        RobotLog.ii(TAG, "Hood angle LUT created with 8 data points (servo range 0.3-1.0)");

        // Test LUTs
        double testDistance = 60.0;
        double testRPM = RPM.get(testDistance);
        double testAngle = angle.get(testDistance);
        RobotLog.ii(TAG, "LUT test - Distance: %.1f -> RPM: %.1f, Angle: %.3f", testDistance, testRPM, testAngle);

        long elapsedTime = System.currentTimeMillis() - startTime;
        RobotLog.ii(TAG, "=== Shooter subsystem initialization completed in %dms ===", elapsedTime);

        // Start monitoring
        periodicTimer.reset();
        RobotLog.ii(TAG, "Shooter periodic monitoring started");
    }

    public Command flywheel (boolean on) {
        return new InstantCommand(() -> {
            if (flywheelOn != on) {
                flywheelOn = on;
                RobotLog.ii(TAG, "Flywheel state changed to: %s", on ? "ON" : "OFF");
            }
        });
    }

    @Override
    public void periodic() {
        periodicCount++;
        long currentTime = System.currentTimeMillis();

        try {
            Pose robot = poseSupplier.get();
            double presentVoltage = volt.getVoltage();

            // Log voltage warnings
            if (presentVoltage < 11.5) {
                RobotLog.ww(TAG, "Low battery voltage: %.2fV", presentVoltage);
            }

            double robotX = robot.getX();
            double robotY = robot.getY();
            double robotHeading = robot.getHeading();

            // Calculate targeting geometry
            double dx = shooterX - robotX;
            double dy = shooterY - robotY;
            double distance = Math.sqrt(dx*dx + dy*dy);
            double targetAngleRad = Math.atan2(dy, dx);
            double targetAngleDeg = Math.toDegrees(targetAngleRad) - Math.toDegrees(robotHeading);

            // Apply angle constraints
            double constrainedAngle = Math.max(targetAngleDeg, -30);
            constrainedAngle = Math.min(constrainedAngle, 240);

            // Log angle limiting if it occurs
            if (constrainedAngle != targetAngleDeg) {
                RobotLog.ww(TAG, "Turret angle constrained: %.1f° -> %.1f°", targetAngleDeg, constrainedAngle);
            }
            targetAngleDeg = constrainedAngle;

            // Turret control
            double turretPos = ((double)turret.getCurrentPosition()) / TICKS_PER_DEGREES;
            double turretError = targetAngleDeg - turretPos;
            double turretPower = controllerTurret.calculate(turretPos, targetAngleDeg);
            turret.set(turretPower / presentVoltage);

            // Log flywheel state changes
            if (flywheelOn != lastFlywheelState) {
                RobotLog.ii(TAG, "Flywheel state transition: %s -> %s",
                    lastFlywheelState ? "ON" : "OFF", flywheelOn ? "ON" : "OFF");
                lastFlywheelState = flywheelOn;
            }

            if (flywheelOn) {
                target = RPM.get(distance);
                double hoodAngle = angle.get(distance);
                hood.set(hoodAngle);

                double vel = shooterb.getVelocity() * (2 * Math.PI / 28);
                double shooterError = target - vel;
                double flywheelPID = controllerShooter.calculate(vel, target);
                flywheelPID = Math.max(-presentVoltage, Math.min(flywheelPID, presentVoltage));

                shootert.set((-1) * flywheelPID / presentVoltage);
                shooterb.set(flywheelPID / presentVoltage);

                // Log shooting parameters every 2 seconds when active
                if (LoggingConfig.ENABLE_PERFORMANCE_LOGGING && currentTime - lastHealthCheck > 2000) {
                    RobotLog.ii(TAG, "=== Shooting Status ===");
                    RobotLog.ii(TAG, "Robot: (%.1f, %.1f) -> Target: (%.1f, %.1f)", robotX, robotY, shooterX, shooterY);
                    RobotLog.ii(TAG, "Distance: %.1f, Target RPM: %.1f, Current vel: %.1f", distance, target, vel);
                    RobotLog.ii(TAG, "Shooter error: %.1f RPM, PID output: %.3f", shooterError, flywheelPID);
                    RobotLog.ii(TAG, "Turret: Current %.1f°, Target %.1f°, Error %.1f°", turretPos, targetAngleDeg, turretError);
                    RobotLog.ii(TAG, "Hood angle: %.3f, Turret power: %.3f", hoodAngle, turretPower);
                    lastHealthCheck = currentTime;
                }
            } else {
                shooterb.set(0);
                shootert.set(0);
            }

            // Periodic health check every 5 seconds
            if (periodicCount % 250 == 0) { // ~5 seconds at 50Hz
                logSystemHealth(distance, targetAngleDeg, turretPos, presentVoltage);
            }

            // Keep original logging for compatibility
            Log.d("Turret angle", String.valueOf(targetAngleDeg));

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error in shooter periodic() cycle %d: %s", periodicCount, e.getMessage());
        }
    }

    /**
     * Log system health metrics
     */
    private void logSystemHealth(double distance, double targetAngle, double currentAngle, double voltage) {
        RobotLog.ii(TAG, "=== Shooter Health Check ===");
        RobotLog.ii(TAG, "Periodic cycles completed: %d", periodicCount);
        RobotLog.ii(TAG, "Runtime: %.2f seconds", periodicTimer.seconds());
        RobotLog.ii(TAG, "Battery voltage: %.2fV", voltage);
        RobotLog.ii(TAG, "Distance to target: %.1f units", distance);
        RobotLog.ii(TAG, "Turret position: %.1f° (target: %.1f°)", currentAngle, targetAngle);
        RobotLog.ii(TAG, "Flywheel active: %s", flywheelOn ? "YES" : "NO");

        // Check for potential issues
        if (Math.abs(targetAngle - currentAngle) > 10.0) {
            RobotLog.ww(TAG, "Large turret angle error detected: %.1f°", Math.abs(targetAngle - currentAngle));
        }

        if (distance > 150.0) {
            RobotLog.ww(TAG, "Target distance is very far: %.1f units", distance);
        }
    }
}