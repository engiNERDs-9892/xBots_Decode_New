package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple autonomous example for goBILDA motor setup
 * - Uses precision motors (312 RPM) for accurate driving
 * - Uses high speed motors (6000+ RPM) for mechanisms
 * - Basic movement and mechanism control
 */
@Autonomous(name="Simple Auto - goBILDA", group="Linear Opmode")
public class SimpleAutonomous extends LinearOpMode {

    // Declare OpMode members
    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    
    // Motor speeds for autonomous
    private static final double DRIVE_SPEED = 0.8;      // 312 RPM precision motors
    private static final double ARM_SPEED = 0.3;        // 6000+ RPM high speed motor (limited)
    private static final double LIFT_SPEED = 0.4;       // 6000+ RPM high speed motor (limited)
    
    @Override
    public void runOpMode() {

        // Initialize the drive system variables
        robot.init(hardwareMap);
        
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Drive Motors", "312 RPM precision motors");
        telemetry.addData("Mechanism Motors", "6000+ RPM high speed motors");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped
        
        // Step 1: Initialize robot position
        robot.setClawPosition(0.0);  // Open claw
        robot.setWristPosition(0.5); // Center wrist
        sleep(500);
        
        // Step 2: Lift arm to safe position using high speed motor (limited power)
        telemetry.addData("Path", "Step 2: Raising arm");
        telemetry.update();
        robot.setArmPower(ARM_SPEED);
        sleep(1000);  // Run for 1 second
        robot.setArmPower(0);
        
        // Step 3: Drive forward using precision 312 RPM motors
        telemetry.addData("Path", "Step 3: Driving forward");
        telemetry.update();
        robot.setTankDrive(DRIVE_SPEED, DRIVE_SPEED);
        sleep(2000);  // Drive for 2 seconds
        robot.setTankDrive(0, 0);
        
        // Step 4: Turn right
        telemetry.addData("Path", "Step 4: Turning right");
        telemetry.update();
        robot.setTankDrive(DRIVE_SPEED, -DRIVE_SPEED);
        sleep(1000);  // Turn for 1 second
        robot.setTankDrive(0, 0);
        
        // Step 5: Drive forward again
        telemetry.addData("Path", "Step 5: Driving forward again");
        telemetry.update();
        robot.setTankDrive(DRIVE_SPEED, DRIVE_SPEED);
        sleep(1500);  // Drive for 1.5 seconds
        robot.setTankDrive(0, 0);
        
        // Step 6: Lower arm and activate intake (high speed motors with limited power)
        telemetry.addData("Path", "Step 6: Lowering arm and intake");
        telemetry.update();
        robot.setArmPower(-ARM_SPEED);
        robot.setIntakePower(0.6);  // Limited power for high speed motor
        sleep(1500);
        robot.setArmPower(0);
        robot.setIntakePower(0);
        
        // Step 7: Close claw to grab object
        telemetry.addData("Path", "Step 7: Closing claw");
        telemetry.update();
        robot.setClawPosition(1.0);  // Close claw
        sleep(500);
        
        // Step 8: Raise lift using high speed motor (limited power)
        telemetry.addData("Path", "Step 8: Raising lift");
        telemetry.update();
        robot.setLiftPower(LIFT_SPEED);
        sleep(2000);
        robot.setLiftPower(0);
        
        // Step 9: Drive backward to starting position
        telemetry.addData("Path", "Step 9: Returning to start");
        telemetry.update();
        robot.setTankDrive(-DRIVE_SPEED, -DRIVE_SPEED);
        sleep(2000);
        robot.setTankDrive(0, 0);
        
        // Step 10: Turn back to original heading
        telemetry.addData("Path", "Step 10: Final turn");
        telemetry.update();
        robot.setTankDrive(-DRIVE_SPEED, DRIVE_SPEED);
        sleep(1000);
        robot.setTankDrive(0, 0);
        
        telemetry.addData("Path", "Complete");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.update();
        
        sleep(1000);  // Pause to show final telemetry
    }
    
    /**
     * Method to drive forward for a specific distance using time
     * @param speed Motor speed (0.0 to 1.0)
     * @param timeMs Time to drive in milliseconds
     */
    public void driveForTime(double speed, long timeMs) {
        robot.setTankDrive(speed, speed);
        sleep(timeMs);
        robot.setTankDrive(0, 0);
    }
    
    /**
     * Method to turn for a specific amount of time
     * @param speed Motor speed (0.0 to 1.0)
     * @param timeMs Time to turn in milliseconds
     */
    public void turnForTime(double speed, long timeMs) {
        robot.setTankDrive(speed, -speed);
        sleep(timeMs);
        robot.setTankDrive(0, 0);
    }
}