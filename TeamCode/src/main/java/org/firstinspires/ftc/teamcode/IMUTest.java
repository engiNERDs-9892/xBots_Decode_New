package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * IMU Test OpMode for internal Control Hub/Expansion Hub IMU
 * Tests the built-in BNO055 IMU functionality
 */
@TeleOp(name="IMU Test - Internal", group="Test")
public class IMUTest extends LinearOpMode {

    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    private boolean lastResetButton = false;

    @Override
    public void runOpMode() {
        
        // Initialize the hardware variables
        robot.init(hardwareMap);
        
        telemetry.addData("Status", "IMU Test Ready");
        telemetry.addData("Purpose", "Test internal Control Hub/Expansion Hub IMU");
        telemetry.addData("IMU Available", robot.isIMUAvailable());
        telemetry.addData("Controls", "A = Reset Heading, Drive to test");
        
        if (!robot.isIMUAvailable()) {
            telemetry.addData("ERROR", "IMU not found!");
            telemetry.addData("Check", "Robot configuration has 'imu' device");
        }
        
        telemetry.update();

        // Wait for the game to start
        waitForStart();
        runtime.reset();

        // Run until the end of the match
        while (opModeIsActive()) {
            
            // ================== RESET HEADING ==================
            
            boolean currentResetButton = gamepad1.a;
            if (currentResetButton && !lastResetButton) {
                robot.resetIMUHeading();
                telemetry.addData("Action", "Heading Reset!");
            }
            lastResetButton = currentResetButton;
            
            // ================== BASIC DRIVE FOR TESTING ==================
            
            // Simple tank drive to test IMU while moving
            double drive = -gamepad1.left_stick_y * 0.5;
            double turn = gamepad1.right_stick_x * 0.3;
            
            robot.setTankDrive(drive + turn, drive - turn);
            
            // ================== TELEMETRY ==================
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("IMU Status", robot.isIMUAvailable() ? "CONNECTED" : "NOT FOUND");
            
            if (robot.isIMUAvailable()) {
                telemetry.addData("Heading (Degrees)", "%.1f°", robot.getHeadingDegrees());
                telemetry.addData("Heading (Radians)", "%.3f rad", robot.getHeadingRadians());
                
                // Show heading in a more visual way
                double heading = robot.getHeadingDegrees();
                String direction = "";
                if (heading >= -22.5 && heading < 22.5) direction = "FORWARD ↑";
                else if (heading >= 22.5 && heading < 67.5) direction = "FORWARD-RIGHT ↗";
                else if (heading >= 67.5 && heading < 112.5) direction = "RIGHT →";
                else if (heading >= 112.5 && heading < 157.5) direction = "BACK-RIGHT ↘";
                else if (heading >= 157.5 || heading < -157.5) direction = "BACK ↓";
                else if (heading >= -157.5 && heading < -112.5) direction = "BACK-LEFT ↙";
                else if (heading >= -112.5 && heading < -67.5) direction = "LEFT ←";
                else if (heading >= -67.5 && heading < -22.5) direction = "FORWARD-LEFT ↖";
                
                telemetry.addData("Robot Facing", direction);
            } else {
                telemetry.addData("ERROR", "Configure IMU as 'imu' in robot config");
            }
            
            telemetry.addLine();
            telemetry.addData("Drive", "Left stick = forward/back");
            telemetry.addData("Turn", "Right stick = turn to test IMU");
            telemetry.addData("Reset", "A button = reset heading to 0°");
            
            telemetry.addLine();
            telemetry.addData("Drive Motors", "L:%.2f R:%.2f", 
                            (robot.leftFrontDrive.getPower() + robot.leftBackDrive.getPower()) / 2,
                            (robot.rightFrontDrive.getPower() + robot.rightBackDrive.getPower()) / 2);
            
            telemetry.update();
        }
    }
}