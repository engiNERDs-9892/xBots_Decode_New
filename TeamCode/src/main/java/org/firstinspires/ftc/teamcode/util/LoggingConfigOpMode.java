package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Configuration OpMode for quickly switching logging modes during competition.
 *
 * This OpMode allows teams to quickly adjust logging levels without modifying code:
 * - Development Mode: Maximum logging for debugging issues
 * - Competition Mode: Minimal logging for best performance
 * - Testing Mode: Balanced logging for practice matches
 * - Minimal Mode: Only critical errors and warnings
 *
 * Usage:
 * 1. Run this OpMode between matches to configure logging
 * 2. Use gamepad buttons to select desired mode
 * 3. Configuration persists until changed or robot reboot
 *
 * adb logcat monitoring:
 * - adb logcat | grep "LoggingConfig\|Performance"
 * - Check current mode in any OpMode status logs
 */
@Config
@TeleOp(name="⚙️ Logging Configuration", group="Utility")
public class LoggingConfigOpMode extends LinearOpMode {
    private static final String TAG = "LoggingConfig";

    @Override
    public void runOpMode() {
        RobotLog.ii(TAG, "=== Logging Configuration OpMode Started ===");

        telemetry.addData("🚨", "LOGGING CONFIGURATION");
        telemetry.addData("", "");
        telemetry.addData("Current Mode", getCurrentModeDescription());
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("🔧 A", "Development Mode (Max Logging)");
        telemetry.addData("🏆 B", "Competition Mode (Min Logging)");
        telemetry.addData("🧪 X", "Testing Mode (Balanced)");
        telemetry.addData("⚡ Y", "Minimal Mode (Errors Only)");
        telemetry.addData("", "");
        telemetry.addData("📊 DPAD_UP", "Show Current Settings");
        telemetry.addData("🔄 DPAD_DOWN", "Reset to Development");
        telemetry.addData("", "");
        telemetry.addData("Current Config", LoggingConfig.getConfigurationSummary());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Mode selection buttons
            if (gamepad1.a) {
                setDevelopmentMode();
                sleep(500); // Debounce
            } else if (gamepad1.b) {
                setCompetitionMode();
                sleep(500);
            } else if (gamepad1.x) {
                setTestingMode();
                sleep(500);
            } else if (gamepad1.y) {
                setMinimalMode();
                sleep(500);
            } else if (gamepad1.dpad_up) {
                showDetailedSettings();
                sleep(1000);
            } else if (gamepad1.dpad_down) {
                resetToDefault();
                sleep(500);
            }

            // Update telemetry
            telemetry.addData("🚨", "LOGGING CONFIGURATION");
            telemetry.addData("", "");
            telemetry.addData("Current Mode", getCurrentModeDescription());
            telemetry.addData("Runtime", "%.1f seconds", getRuntime());
            telemetry.addData("", "");
            telemetry.addData("Quick Status", "");
            telemetry.addData("Debug", LoggingConfig.DEBUG_MODE ? "✅ ON" : "❌ OFF");
            telemetry.addData("Performance", LoggingConfig.ENABLE_PERFORMANCE_LOGGING ? "✅ ON" : "❌ OFF");
            telemetry.addData("Teleop", LoggingConfig.ENABLE_TELEOP_LOGGING ? "✅ ON" : "❌ OFF");
            telemetry.addData("Shooter", LoggingConfig.ENABLE_SHOOTER_LOGGING ? "✅ ON" : "❌ OFF");
            telemetry.addData("", "");
            telemetry.addData("Impact", getPerformanceImpactEstimate());
            telemetry.update();

            sleep(50);
        }

        RobotLog.ii(TAG, "=== Logging Configuration OpMode Ended ===");
        RobotLog.ii(TAG, "Final configuration: %s", LoggingConfig.getConfigurationSummary());
    }

    private void setDevelopmentMode() {
        LoggingConfig.setDevelopmentMode();
        RobotLog.ii(TAG, "🔧 DEVELOPMENT MODE ACTIVATED - Maximum logging enabled");
        telemetry.speak("Development mode activated");
    }

    private void setCompetitionMode() {
        LoggingConfig.setCompetitionMode();
        RobotLog.ii(TAG, "🏆 COMPETITION MODE ACTIVATED - Minimal logging for best performance");
        telemetry.speak("Competition mode activated");
    }

    private void setTestingMode() {
        LoggingConfig.setTestingMode();
        RobotLog.ii(TAG, "🧪 TESTING MODE ACTIVATED - Balanced logging for practice");
        telemetry.speak("Testing mode activated");
    }

    private void setMinimalMode() {
        LoggingConfig.setMinimalMode();
        RobotLog.ii(TAG, "⚡ MINIMAL MODE ACTIVATED - Only critical errors");
        telemetry.speak("Minimal mode activated");
    }

    private void resetToDefault() {
        LoggingConfig.setDevelopmentMode();
        RobotLog.ii(TAG, "🔄 RESET TO DEFAULT - Development mode restored");
        telemetry.speak("Configuration reset");
    }

    private void showDetailedSettings() {
        RobotLog.ii(TAG, "=== DETAILED LOGGING CONFIGURATION ===");
        RobotLog.ii(TAG, "DEBUG_MODE: %s", LoggingConfig.DEBUG_MODE);
        RobotLog.ii(TAG, "ENABLE_PERFORMANCE_LOGGING: %s", LoggingConfig.ENABLE_PERFORMANCE_LOGGING);
        RobotLog.ii(TAG, "ENABLE_DEBUG_LOGGING: %s", LoggingConfig.ENABLE_DEBUG_LOGGING);
        RobotLog.ii(TAG, "ENABLE_INIT_LOGGING: %s", LoggingConfig.ENABLE_INIT_LOGGING);
        RobotLog.ii(TAG, "ENABLE_WARNING_LOGGING: %s", LoggingConfig.ENABLE_WARNING_LOGGING);
        RobotLog.ii(TAG, "ENABLE_AUTONOMOUS_LOGGING: %s", LoggingConfig.ENABLE_AUTONOMOUS_LOGGING);
        RobotLog.ii(TAG, "ENABLE_TELEOP_LOGGING: %s", LoggingConfig.ENABLE_TELEOP_LOGGING);
        RobotLog.ii(TAG, "ENABLE_SHOOTER_LOGGING: %s", LoggingConfig.ENABLE_SHOOTER_LOGGING);
        RobotLog.ii(TAG, "ENABLE_DRIVETRAIN_LOGGING: %s", LoggingConfig.ENABLE_DRIVETRAIN_LOGGING);
        RobotLog.ii(TAG, "ENABLE_VISION_LOGGING: %s", LoggingConfig.ENABLE_VISION_LOGGING);
        RobotLog.ii(TAG, "=== END DETAILED CONFIGURATION ===");
    }

    private String getCurrentModeDescription() {
        if (LoggingConfig.DEBUG_MODE && LoggingConfig.ENABLE_DEBUG_LOGGING &&
            LoggingConfig.ENABLE_TELEOP_LOGGING && LoggingConfig.ENABLE_VISION_LOGGING) {
            return "🔧 DEVELOPMENT (Max Logging)";
        } else if (!LoggingConfig.DEBUG_MODE && !LoggingConfig.ENABLE_DEBUG_LOGGING &&
                   !LoggingConfig.ENABLE_TELEOP_LOGGING && !LoggingConfig.ENABLE_VISION_LOGGING) {
            return "🏆 COMPETITION (Min Logging)";
        } else if (LoggingConfig.DEBUG_MODE && LoggingConfig.ENABLE_DEBUG_LOGGING &&
                   LoggingConfig.ENABLE_TELEOP_LOGGING && !LoggingConfig.ENABLE_VISION_LOGGING) {
            return "🧪 TESTING (Balanced)";
        } else if (!LoggingConfig.DEBUG_MODE && !LoggingConfig.ENABLE_PERFORMANCE_LOGGING) {
            return "⚡ MINIMAL (Errors Only)";
        } else {
            return "🔀 CUSTOM (Manual Config)";
        }
    }

    private String getPerformanceImpactEstimate() {
        int activeLoggers = 0;
        if (LoggingConfig.DEBUG_MODE) activeLoggers += 3;
        if (LoggingConfig.ENABLE_PERFORMANCE_LOGGING) activeLoggers += 1;
        if (LoggingConfig.ENABLE_DEBUG_LOGGING) activeLoggers += 2;
        if (LoggingConfig.ENABLE_TELEOP_LOGGING) activeLoggers += 2;
        if (LoggingConfig.ENABLE_SHOOTER_LOGGING) activeLoggers += 2;
        if (LoggingConfig.ENABLE_DRIVETRAIN_LOGGING) activeLoggers += 1;
        if (LoggingConfig.ENABLE_VISION_LOGGING) activeLoggers += 3;

        if (activeLoggers <= 3) {
            return "🟢 LOW (~1-2ms overhead)";
        } else if (activeLoggers <= 8) {
            return "🟡 MEDIUM (~2-5ms overhead)";
        } else {
            return "🔴 HIGH (~5-10ms overhead)";
        }
    }
}
