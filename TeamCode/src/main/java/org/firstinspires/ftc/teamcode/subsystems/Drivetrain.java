package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;

import org.firstinspires.ftc.teamcode.util.LoggingConfig;

public class Drivetrain extends SubsystemBase {
    private static final String TAG = "Drivetrain";


    public static Follower follower;
    private boolean teleop = false;

    // Logging and monitoring
    private ElapsedTime driveTimer = new ElapsedTime();
    private int periodicCount = 0;
    private long lastStatusLog = 0;
    private Pose lastPose = new Pose(0, 0, 0);
    private double totalDistanceTraveled = 0.0;

    public Drivetrain(final HardwareMap hMap, boolean teleop, Pose start) {
        RobotLog.ii(TAG, "=== Drivetrain subsystem initialization started ===");
        long initStartTime = System.currentTimeMillis();

        try {
            RobotLog.ii(TAG, "Creating Pedro Pathing follower...");
            follower = Constants.createFollower(hMap);
            RobotLog.ii(TAG, "Follower created successfully");

            RobotLog.ii(TAG, "Setting starting pose: (%.2f, %.2f, %.2f°)",
                start.getX(), start.getY(), Math.toDegrees(start.getHeading()));
            follower.setStartingPose(start);
            lastPose = start;

            this.teleop = teleop;
            RobotLog.ii(TAG, "Drive mode: %s", teleop ? "TELEOP" : "AUTONOMOUS");

            if (teleop) {
                follower.startTeleOpDrive();
                RobotLog.ii(TAG, "TeleOp drive mode activated");
            }

            follower.update();
            RobotLog.ii(TAG, "Initial follower update completed");

            driveTimer.reset();
            long elapsedTime = System.currentTimeMillis() - initStartTime;
            RobotLog.ii(TAG, "=== Drivetrain subsystem initialization completed in %dms ===", elapsedTime);

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error during drivetrain initialization: %s", e.getMessage());
            RobotLog.ee(TAG, "Stack trace: %s", android.util.Log.getStackTraceString(e));
            throw e;
        }
    }

    @Override
    public void periodic() {
        periodicCount++;
        long currentTime = System.currentTimeMillis();

        try {
            // Get current pose
            Pose currentPose = follower.getPose();

            // Update Memory with current position
            Memory.robotHeading = follower.getHeading();
            Memory.robotAutoX = currentPose.getX();
            Memory.robotAutoY = currentPose.getY();

            // Calculate distance traveled since last update
            double deltaX = currentPose.getX() - lastPose.getX();
            double deltaY = currentPose.getY() - lastPose.getY();
            double deltaDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            totalDistanceTraveled += deltaDistance;

            follower.update();

            // Log position and movement every 5 seconds
            if (LoggingConfig.ENABLE_DRIVETRAIN_LOGGING && currentTime - lastStatusLog > 5000) {
                logDrivetrainStatus(currentPose);
                lastStatusLog = currentTime;
            }

            // Log significant movement
            if (LoggingConfig.ENABLE_DEBUG_LOGGING && deltaDistance > 5.0) { // More than 5 units moved
                RobotLog.dd(TAG, "Significant movement - Delta: (%.2f, %.2f), Distance: %.2f",
                    deltaX, deltaY, deltaDistance);
            }

            // Update last pose for next iteration
            lastPose = currentPose;

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error in drivetrain periodic() cycle %d: %s", periodicCount, e.getMessage());
        }
    }

    /**
     * Log comprehensive drivetrain status
     */
    private void logDrivetrainStatus(Pose currentPose) {
        RobotLog.ii(TAG, "=== Drivetrain Status ===");
        RobotLog.ii(TAG, "Mode: %s", teleop ? "TeleOp" : "Autonomous");
        RobotLog.ii(TAG, "Current pose: (%.2f, %.2f, %.2f°)",
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        RobotLog.ii(TAG, "Runtime: %.2f seconds", driveTimer.seconds());
        RobotLog.ii(TAG, "Periodic cycles: %d", periodicCount);
        RobotLog.ii(TAG, "Total distance traveled: %.2f units", totalDistanceTraveled);

        // Log follower status
        if (follower.isBusy()) {
            RobotLog.ii(TAG, "Follower status: BUSY (following path)");
        } else {
            RobotLog.ii(TAG, "Follower status: IDLE");
        }

        RobotLog.ii(TAG, "Memory sync - X: %.2f, Y: %.2f, Heading: %.2f°",
            Memory.robotAutoX, Memory.robotAutoY, Math.toDegrees(Memory.robotHeading));
    }

    /**
     * Get current drivetrain performance metrics
     */
    public void logPerformanceMetrics() {
        try {
            double avgCycleTime = (driveTimer.milliseconds() * 1000) / periodicCount;
            RobotLog.ii(TAG, "=== Drivetrain Performance ===");
            RobotLog.ii(TAG, "Average cycle time: %.2fms over %d cycles", avgCycleTime, periodicCount);
            RobotLog.ii(TAG, "Total runtime: %.2f seconds", driveTimer.seconds());
            RobotLog.ii(TAG, "Distance per second: %.2f units/sec",
                totalDistanceTraveled / driveTimer.seconds());
        } catch (Exception e) {
            RobotLog.ee(TAG, "Error calculating performance metrics: %s", e.getMessage());
        }
    }
}
