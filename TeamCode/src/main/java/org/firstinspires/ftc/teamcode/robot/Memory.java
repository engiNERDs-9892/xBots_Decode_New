package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.RobotLog;

public class Memory {
    private static final String TAG = "Memory";

    public static boolean allianceRed = true;
    public static double robotAutoX;
    public static double robotAutoY;
    public static double robotHeading;
    public static Pose robotPose = new Pose(72, 72, Math.toRadians(90));
    public static boolean autoRan = false;

    // State tracking for logging
    private static boolean lastAllianceRed = true;
    private static double lastRobotAutoX = 0;
    private static double lastRobotAutoY = 0;
    private static double lastRobotHeading = 0;
    private static boolean lastAutoRan = false;
    private static long lastStateLogTime = 0;

    static {
        RobotLog.ii(TAG, "Memory class initialized with default values");
        RobotLog.ii(TAG, "Default alliance: %s", allianceRed ? "RED" : "BLUE");
        RobotLog.ii(TAG, "Default robot pose: (%.1f, %.1f, %.1f°)",
            robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
        logMemoryState("INITIALIZATION");
    }

    /**
     * Update alliance selection with logging
     */
    public static void setAlliance(boolean isRed) {
        if (allianceRed != isRed) {
            RobotLog.ii(TAG, "Alliance changed: %s -> %s",
                allianceRed ? "RED" : "BLUE", isRed ? "RED" : "BLUE");
            lastAllianceRed = allianceRed;
            allianceRed = isRed;
            logMemoryState("ALLIANCE_CHANGE");
        }
    }

    /**
     * Update robot position with logging (when significant change occurs)
     */
    public static void updateRobotPosition(double x, double y, double heading) {
        double deltaX = Math.abs(x - lastRobotAutoX);
        double deltaY = Math.abs(y - lastRobotAutoY);
        double deltaHeading = Math.abs(heading - lastRobotHeading);

        // Log if position changed significantly
        if (deltaX > 5.0 || deltaY > 5.0 || deltaHeading > Math.toRadians(15)) {
            RobotLog.ii(TAG, "Robot position updated: (%.2f, %.2f, %.2f°) -> (%.2f, %.2f, %.2f°)",
                lastRobotAutoX, lastRobotAutoY, Math.toDegrees(lastRobotHeading),
                x, y, Math.toDegrees(heading));
        }

        lastRobotAutoX = robotAutoX;
        lastRobotAutoY = robotAutoY;
        lastRobotHeading = robotHeading;

        robotAutoX = x;
        robotAutoY = y;
        robotHeading = heading;

        // Update robotPose as well
        robotPose = new Pose(x, y, heading);
    }

    /**
     * Set autonomous completion status with logging
     */
    public static void setAutoCompleted(boolean completed) {
        if (autoRan != completed) {
            RobotLog.ii(TAG, "Autonomous status changed: %s -> %s",
                autoRan ? "COMPLETED" : "NOT_RUN", completed ? "COMPLETED" : "NOT_RUN");
            lastAutoRan = autoRan;
            autoRan = completed;
            logMemoryState("AUTO_STATUS_CHANGE");
        }
    }

    /**
     * Log current memory state
     */
    public static void logMemoryState(String trigger) {
        RobotLog.ii(TAG, "=== Memory State (%s) ===", trigger);
        RobotLog.ii(TAG, "Alliance: %s", allianceRed ? "RED" : "BLUE");
        RobotLog.ii(TAG, "Robot Position: (%.2f, %.2f, %.2f°)", robotAutoX, robotAutoY, Math.toDegrees(robotHeading));
        RobotLog.ii(TAG, "Robot Pose: (%.2f, %.2f, %.2f°)",
            robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
        RobotLog.ii(TAG, "Autonomous Status: %s", autoRan ? "COMPLETED" : "NOT_RUN");
        RobotLog.ii(TAG, "=== End Memory State ===");
    }

    /**
     * Periodic memory monitoring (call from main loops)
     */
    public static void periodicLogging() {
        long currentTime = System.currentTimeMillis();

        // Log memory state every 10 seconds
        if (currentTime - lastStateLogTime > 10000) {
            logMemoryState("PERIODIC_CHECK");
            lastStateLogTime = currentTime;
        }

        // Check for unexpected state changes
        checkStateConsistency();
    }

    /**
     * Check for state consistency and warn about unexpected changes
     */
    private static void checkStateConsistency() {
        // Check if robotPose is consistent with individual position variables
        double poseDeltaX = Math.abs(robotPose.getX() - robotAutoX);
        double poseDeltaY = Math.abs(robotPose.getY() - robotAutoY);
        double poseDeltaHeading = Math.abs(robotPose.getHeading() - robotHeading);

        if (poseDeltaX > 0.1 || poseDeltaY > 0.1 || poseDeltaHeading > 0.01) {
            RobotLog.ww(TAG, "Memory inconsistency detected!");
            RobotLog.ww(TAG, "robotPose: (%.2f, %.2f, %.2f°)",
                robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
            RobotLog.ww(TAG, "Individual vars: (%.2f, %.2f, %.2f°)",
                robotAutoX, robotAutoY, Math.toDegrees(robotHeading));

            // Auto-correct by syncing robotPose to individual variables
            robotPose = new Pose(robotAutoX, robotAutoY, robotHeading);
            RobotLog.ii(TAG, "Memory auto-corrected to individual variables");
        }
    }

    /**
     * Reset memory to default state with logging
     */
    public static void reset() {
        RobotLog.ii(TAG, "Resetting Memory to default state");
        logMemoryState("PRE_RESET");

        allianceRed = true;
        robotAutoX = 72;
        robotAutoY = 72;
        robotHeading = Math.toRadians(90);
        robotPose = new Pose(72, 72, Math.toRadians(90));
        autoRan = false;

        logMemoryState("POST_RESET");
    }
}
