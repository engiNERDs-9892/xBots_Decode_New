package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.util.LoggingConfig;

public class Intake extends SubsystemBase {
    private static final String TAG = "Intake";

    private final MotorEx intake;
    private final ServoEx latch;

    // State tracking for logging
    private double lastMotorPower = 0.0;
    private double lastLatchPosition = -1.0;
    private ElapsedTime stateTimer = new ElapsedTime();
    private int commandCount = 0;

    public Intake(final HardwareMap hMap) {
        RobotLog.ii(TAG, "=== Intake subsystem initialization started ===");
        long startTime = System.currentTimeMillis();

        try {
            intake = new MotorEx(hMap, "intake");
            RobotLog.ii(TAG, "Intake motor initialized successfully");

            latch = new ServoEx(hMap, "latch");
            RobotLog.ii(TAG, "Latch servo initialized successfully");

            intake.setRunMode(MotorEx.RunMode.RawPower);
            RobotLog.ii(TAG, "Intake motor set to RawPower mode");

            // Initialize state tracking
            lastLatchPosition = latch.get();
            stateTimer.reset();

            long elapsedTime = System.currentTimeMillis() - startTime;
            RobotLog.ii(TAG, "=== Intake subsystem initialization completed in %dms ===", elapsedTime);
            RobotLog.ii(TAG, "Initial latch position: %.3f", lastLatchPosition);

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error during intake initialization: %s", e.getMessage());
            RobotLog.ee(TAG, "Stack trace: %s", android.util.Log.getStackTraceString(e));
            throw e;
        }
    }

    public Command collect() {
        return new InstantCommand(() -> {
            commandCount++;
            double newPower = 1.0;
            logMotorStateChange("COLLECT", lastMotorPower, newPower);
            intake.set(newPower);
            lastMotorPower = newPower;
        });
    }

    public Command reverse() {
        return new InstantCommand(() -> {
            commandCount++;
            double newPower = -1.0;
            logMotorStateChange("REVERSE", lastMotorPower, newPower);
            intake.set(newPower);
            lastMotorPower = newPower;
        });
    }

    public Command stop() {
        return new InstantCommand(() -> {
            commandCount++;
            double newPower = 0.0;
            logMotorStateChange("STOP", lastMotorPower, newPower);
            intake.set(newPower);
            lastMotorPower = newPower;
        });
    }

    public Command open() {
        return new InstantCommand(() -> {
            commandCount++;
            double newPosition = 1.0;
            logLatchStateChange("OPEN", lastLatchPosition, newPosition);
            latch.set(newPosition);
            lastLatchPosition = newPosition;
        });
    }

    public Command close() {
        return new InstantCommand(() -> {
            commandCount++;
            double newPosition = 0.0;
            logLatchStateChange("CLOSE", lastLatchPosition, newPosition);
            latch.set(newPosition);
            lastLatchPosition = newPosition;
        });
    }

    /**
     * Log motor state changes
     */
    private void logMotorStateChange(String command, double oldPower, double newPower) {
        if (LoggingConfig.DEBUG_MODE && oldPower != newPower) {
            RobotLog.ii(TAG, "Motor %s: %.1f -> %.1f (Command #%d, Runtime: %.2fs)",
                command, oldPower, newPower, commandCount, stateTimer.seconds());
        } else if (LoggingConfig.ENABLE_DEBUG_LOGGING) {
            RobotLog.dd(TAG, "Motor %s: Already at %.1f (Command #%d)", command, newPower, commandCount);
        }
    }

    /**
     * Log latch state changes
     */
    private void logLatchStateChange(String command, double oldPosition, double newPosition) {
        if (LoggingConfig.DEBUG_MODE && Math.abs(oldPosition - newPosition) > 0.01) {
            RobotLog.ii(TAG, "Latch %s: %.3f -> %.3f (Command #%d, Runtime: %.2fs)",
                command, oldPosition, newPosition, commandCount, stateTimer.seconds());
        } else if (LoggingConfig.ENABLE_DEBUG_LOGGING) {
            RobotLog.dd(TAG, "Latch %s: Already at %.3f (Command #%d)", command, newPosition, commandCount);
        }
    }

    @Override
    public void periodic() {
        // Monitor for unexpected state changes (safety check)
        try {
            double currentMotorPower = intake.get();
            double currentLatchPos = latch.get();

            // Check for unexpected motor power changes
            if (Math.abs(currentMotorPower - lastMotorPower) > 0.1) {
                RobotLog.ww(TAG, "Unexpected motor power change detected: %.3f -> %.3f",
                    lastMotorPower, currentMotorPower);
                lastMotorPower = currentMotorPower;
            }

            // Check for unexpected latch position changes
            if (Math.abs(currentLatchPos - lastLatchPosition) > 0.05) {
                RobotLog.ww(TAG, "Unexpected latch position change detected: %.3f -> %.3f",
                    lastLatchPosition, currentLatchPos);
                lastLatchPosition = currentLatchPos;
            }

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error in intake periodic monitoring: %s", e.getMessage());
        }
    }

    /**
     * Get current intake system status for debugging
     */
    public void logStatus() {
        try {
            RobotLog.ii(TAG, "=== Intake Status ===");
            RobotLog.ii(TAG, "Motor power: %.3f", intake.get());
            RobotLog.ii(TAG, "Motor velocity: %.1f ticks/sec", intake.getVelocity());
            RobotLog.ii(TAG, "Latch position: %.3f", latch.get());
            RobotLog.ii(TAG, "Commands executed: %d", commandCount);
            RobotLog.ii(TAG, "Runtime: %.2f seconds", stateTimer.seconds());
        } catch (Exception e) {
            RobotLog.ee(TAG, "Error getting intake status: %s", e.getMessage());
        }
    }
}