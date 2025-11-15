package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

/**
 * Centralized logging configuration for all TeamCode files.
 *
 * This class provides global logging control flags that can be easily modified
 * for different scenarios:
 * - Development: All logging enabled for maximum debugging information
 * - Testing: Performance logging enabled, debug logging reduced
 * - Competition: Minimal logging for maximum performance
 *
 * Usage in adb logcat debugging:
 * - View all logs: adb logcat | grep "TurtleRobot\|Teleop\|AutonLinear"
 * - Errors only: adb logcat | grep "E/"
 * - Performance: adb logcat | grep "PERFORMANCE\|Performance"
 *
 * @author TeamCode Enhanced Logging System
 */
@Config
public class LoggingConfig {

    // ==============================================
    // GLOBAL LOGGING CONTROL FLAGS
    // ==============================================

    /**
     * Master debug flag - when false, disables most detailed logging
     * Set to FALSE for competition to minimize performance impact
     */
    public static boolean DEBUG_MODE = true;

    /**
     * Performance logging - system status, loop times, health checks
     * Recommended to keep TRUE even in competition for monitoring
     */
    public static boolean ENABLE_PERFORMANCE_LOGGING = true;

    /**
     * Debug logging - detailed state changes, PID values, minor events
     * Set to FALSE for competition to reduce overhead
     */
    public static boolean ENABLE_DEBUG_LOGGING = true;

    /**
     * Initialization logging - hardware setup and startup events
     * Generally safe to keep TRUE as it only runs during init
     */
    public static boolean ENABLE_INIT_LOGGING = true;

    /**
     * Error logging - always enabled for safety
     * Should NEVER be disabled
     */
    public static final boolean ENABLE_ERROR_LOGGING = true;

    /**
     * Warning logging - battery, overcurrent, constraint violations
     * Recommended to keep TRUE for safety monitoring
     */
    public static boolean ENABLE_WARNING_LOGGING = true;

    // ==============================================
    // SUBSYSTEM-SPECIFIC FLAGS
    // ==============================================

    /**
     * Autonomous logging - path execution, phase tracking
     * Valuable for post-match analysis
     */
    public static boolean ENABLE_AUTONOMOUS_LOGGING = true;

    /**
     * Teleop logging - driver inputs, control loops
     * Can be reduced in competition if performance is critical
     */
    public static boolean ENABLE_TELEOP_LOGGING = true;

    /**
     * Shooter logging - PID performance, targeting calculations
     * Useful for tuning and competition analysis
     */
    public static boolean ENABLE_SHOOTER_LOGGING = true;

    /**
     * Drivetrain logging - movement, positioning, path following
     * Moderate impact, valuable for debugging drive issues
     */
    public static boolean ENABLE_DRIVETRAIN_LOGGING = true;

    /**
     * Vision logging - AprilTag detection, processing performance
     * Can be high impact, disable if vision performance suffers
     */
    public static boolean ENABLE_VISION_LOGGING = true;

    // ==============================================
    // PREDEFINED CONFIGURATIONS
    // ==============================================

    /**
     * Apply development configuration - maximum logging for debugging
     */
    public static void setDevelopmentMode() {
        DEBUG_MODE = true;
        ENABLE_PERFORMANCE_LOGGING = true;
        ENABLE_DEBUG_LOGGING = true;
        ENABLE_INIT_LOGGING = true;
        ENABLE_WARNING_LOGGING = true;
        ENABLE_AUTONOMOUS_LOGGING = true;
        ENABLE_TELEOP_LOGGING = true;
        ENABLE_SHOOTER_LOGGING = true;
        ENABLE_DRIVETRAIN_LOGGING = true;
        ENABLE_VISION_LOGGING = true;
    }

    /**
     * Apply competition configuration - minimal logging for maximum performance
     */
    public static void setCompetitionMode() {
        DEBUG_MODE = false;
        ENABLE_PERFORMANCE_LOGGING = true;  // Keep for monitoring
        ENABLE_DEBUG_LOGGING = false;       // Disable to reduce overhead
        ENABLE_INIT_LOGGING = true;         // Keep for startup issues
        ENABLE_WARNING_LOGGING = true;      // Keep for safety
        ENABLE_AUTONOMOUS_LOGGING = true;   // Keep for match analysis
        ENABLE_TELEOP_LOGGING = false;      // Disable to reduce overhead
        ENABLE_SHOOTER_LOGGING = true;      // Keep for performance monitoring
        ENABLE_DRIVETRAIN_LOGGING = false;  // Disable to reduce overhead
        ENABLE_VISION_LOGGING = false;      // Disable to improve vision performance
    }

    /**
     * Apply testing configuration - balanced logging for practice matches
     */
    public static void setTestingMode() {
        DEBUG_MODE = true;
        ENABLE_PERFORMANCE_LOGGING = true;
        ENABLE_DEBUG_LOGGING = true;
        ENABLE_INIT_LOGGING = true;
        ENABLE_WARNING_LOGGING = true;
        ENABLE_AUTONOMOUS_LOGGING = true;
        ENABLE_TELEOP_LOGGING = true;
        ENABLE_SHOOTER_LOGGING = true;
        ENABLE_DRIVETRAIN_LOGGING = true;
        ENABLE_VISION_LOGGING = false;      // Reduce vision overhead in testing
    }

    /**
     * Apply minimal configuration - only critical logging
     */
    public static void setMinimalMode() {
        DEBUG_MODE = false;
        ENABLE_PERFORMANCE_LOGGING = false;
        ENABLE_DEBUG_LOGGING = false;
        ENABLE_INIT_LOGGING = true;         // Keep for critical startup info
        ENABLE_WARNING_LOGGING = true;      // Keep for safety
        ENABLE_AUTONOMOUS_LOGGING = false;
        ENABLE_TELEOP_LOGGING = false;
        ENABLE_SHOOTER_LOGGING = false;
        ENABLE_DRIVETRAIN_LOGGING = false;
        ENABLE_VISION_LOGGING = false;
    }

    // ==============================================
    // PERFORMANCE THRESHOLDS
    // ==============================================

    /**
     * Loop time warning threshold (milliseconds)
     * Logs warning if loop takes longer than this
     */
    public static int LOOP_TIME_WARNING_MS = 25;

    /**
     * Low battery voltage threshold (volts)
     * Logs warning if voltage drops below this
     */
    public static double LOW_BATTERY_THRESHOLD = 11.5;

    /**
     * Motor overcurrent threshold (amps)
     * Logs warning if motor current exceeds this
     */
    public static double MOTOR_OVERCURRENT_THRESHOLD = 10.0;

    /**
     * Logging frequency divider for periodic messages
     * Higher values = less frequent logging
     */
    public static int PERIODIC_LOG_DIVIDER = 100;  // Every 100 loops (~2 seconds)

    // ==============================================
    // UTILITY METHODS
    // ==============================================

    /**
     * Get current configuration summary for logging
     */
    public static String getConfigurationSummary() {
        return String.format(
            "LogConfig: DEBUG=%s, PERF=%s, DBG=%s, AUTO=%s, TELEOP=%s, SHOOT=%s, DRIVE=%s, VIS=%s",
            DEBUG_MODE, ENABLE_PERFORMANCE_LOGGING, ENABLE_DEBUG_LOGGING,
            ENABLE_AUTONOMOUS_LOGGING, ENABLE_TELEOP_LOGGING, ENABLE_SHOOTER_LOGGING,
            ENABLE_DRIVETRAIN_LOGGING, ENABLE_VISION_LOGGING
        );
    }

    /**
     * Check if any logging is enabled (for performance optimization)
     */
    public static boolean isAnyLoggingEnabled() {
        return DEBUG_MODE || ENABLE_PERFORMANCE_LOGGING || ENABLE_DEBUG_LOGGING ||
               ENABLE_AUTONOMOUS_LOGGING || ENABLE_TELEOP_LOGGING || ENABLE_SHOOTER_LOGGING ||
               ENABLE_DRIVETRAIN_LOGGING || ENABLE_VISION_LOGGING;
    }
}
