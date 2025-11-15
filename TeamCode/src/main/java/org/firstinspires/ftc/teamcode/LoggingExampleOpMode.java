package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

/**
 * Example OpMode demonstrating extensive logging capabilities of TurtleRobot
 *
 * Usage for adb logcat debugging:
 * 1. Connect your Control Hub/Driver Station via USB
 * 2. Run: adb logcat | grep "TurtleRobot"
 * 3. Start this OpMode to see detailed logging output
 *
 * For filtered logging by level:
 * - Errors only: adb logcat | grep "TurtleRobot" | grep "E/"
 * - Warnings: adb logcat | grep "TurtleRobot" | grep "W/"
 * - Info: adb logcat | grep "TurtleRobot" | grep "I/"
 * - Debug: adb logcat | grep "TurtleRobot" | grep "D/"
 */
@TeleOp(name="Logging Example", group="Debug")
public class LoggingExampleOpMode extends LinearOpMode {

    private TurtleRobot robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private int loopCount = 0;

    @Override
    public void runOpMode() {
        // Initialize robot with extensive logging
        robot = new TurtleRobot(this);
        robot.init(hardwareMap);

        // Log system status after initialization
        robot.logSystemStatus();

        telemetry.addData("Status", "Robot initialized with extensive logging");
        telemetry.addData("Info", "Check adb logcat for detailed logs");
        telemetry.addData("Filter", "adb logcat | grep 'TurtleRobot'");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Main control loop with performance monitoring
        while (opModeIsActive()) {
            loopTimer.reset();
            loopCount++;

            // Demonstrate various logging features every 50 loops (~1 second)
            if (loopCount % 50 == 0) {
                demonstrateLogging();
            }

            // Log performance metrics every 100 loops (~2 seconds)
            if (loopCount % 100 == 0) {
                long loopTime = (long) loopTimer.milliseconds();
                double cpuUsage = 0.0; // Would calculate actual CPU usage in real implementation
                robot.logPerformanceMetrics(loopTime, cpuUsage);
            }

            // Basic robot control (example)
            handleGamepadInput();

            // Update telemetry
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.addData("Loop Count", loopCount);
            telemetry.addData("Logging", "Active - check adb logcat");
            telemetry.update();

            // Small delay to prevent overwhelming the logs
            sleep(20);
        }

        // Log shutdown
        robot.logSystemStatus();
    }

    /**
     * Demonstrates various logging capabilities
     */
    private void demonstrateLogging() {
        // System status logging
        robot.logSystemStatus();

        // Shooting calculations example
        double exampleDistance = 75.0;
        double calculatedRPM = 425.0; // Would be calculated from LUT
        double calculatedAngle = 0.45; // Would be calculated from LUT
        robot.logShootingCalculations(exampleDistance, calculatedRPM, calculatedAngle);

        // Turret positioning example
        double robotX = 100.0, robotY = 100.0;
        double targetX = 150.0, targetY = 150.0;
        robot.logTurretPositioning(robotX, robotY, targetX, targetY);

        // Motor health check
        robot.logMotorHealth();

        // Example PID logging (would use actual values in real implementation)
        if (robot.shooterb != null) {
            double currentVel = robot.shooterb.getVelocity();
            double targetRPM = 400.0;
            robot.logShooterPID(currentVel, targetRPM);
        }

        if (robot.turret != null) {
            double currentPos = robot.turret.getCurrentPosition();
            double targetPos = 1000.0; // Example target position
            robot.logTurretPID(currentPos, targetPos);
        }
    }

    /**
     * Handle gamepad input with error logging
     */
    private void handleGamepadInput() {
        try {
            // Example: Shooter control
            if (gamepad1.a) {
                // Would set shooter power here
                robot.logWarning("Shooter", "Shooter activated via gamepad");
            }

            // Example: Turret control
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                // Would set turret power here
                robot.logWarning("Turret", "Manual turret control active");
            }

            // Example: Emergency stop
            if (gamepad1.back && gamepad1.start) {
                robot.logEmergencyStop("Gamepad emergency stop pressed");
                requestOpModeStop();
            }

        } catch (Exception e) {
            robot.logError("GamepadInput", "Error processing gamepad input: " + e.getMessage());
        }
    }
}
