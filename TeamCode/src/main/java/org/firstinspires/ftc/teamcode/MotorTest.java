package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Motor Test OpMode for goBILDA 5203 series motors
 * Test individual motors and verify directions
 * Useful for initial robot setup and troubleshooting
 */
@TeleOp(name="Motor Test - goBILDA", group="Test")
public class MotorTest extends LinearOpMode {

    // Declare OpMode members
    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    
    // Test parameters
    private static final double PRECISION_DRIVE_TEST = 0.6; // Test power for 312 RPM drive motors
    private static final double HIGH_SPEED_TEST = 0.3;      // Test power for 6000+ RPM mechanism motors
    
    // Test state
    private int testMode = 0;  // 0=stopped, 1=drive, 2=arm, 3=intake, 4=lift, 5=servos
    private boolean lastTestButton = false;

    @Override
    public void runOpMode() {
        
        // Initialize the hardware variables
        robot.init(hardwareMap);
        
        telemetry.addData("Status", "Motor Test Ready");
        telemetry.addData("Purpose", "Test goBILDA 5203 motors individually");
        telemetry.addData("Drive Motors", "312 RPM precision motors");
        telemetry.addData("Mechanism Motors", "6000+ RPM high speed motors");
        telemetry.addData("Controls", "A = Next Test, B = Stop All");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // ================== TEST MODE SELECTION ==================
            
            boolean currentTestButton = gamepad1.a;
            if (currentTestButton && !lastTestButton) {
                testMode = (testMode + 1) % 6;  // Cycle through 0-5
            }
            lastTestButton = currentTestButton;
            
            // Stop all motors if B is pressed
            if (gamepad1.b) {
                testMode = 0;
            }
            
            // ================== EXECUTE TESTS ==================
            
            // Stop all motors first
            robot.setDrivePower(0, 0, 0, 0);
            robot.setArmPower(0);
            robot.setIntakePower(0);
            robot.setLiftPower(0);
            
            String currentTest = "STOPPED";
            String instructions = "Press A for next test, B to stop";
            
            switch (testMode) {
                case 0: // All stopped
                    currentTest = "ALL STOPPED";
                    instructions = "Press A to start drive motor test";
                    break;
                    
                case 1: // Test drive motors (312 RPM precision)
                    currentTest = "DRIVE MOTORS (312 RPM Precision)";
                    instructions = "Should drive forward with precise control";
                    robot.setTankDrive(PRECISION_DRIVE_TEST, PRECISION_DRIVE_TEST);
                    break;
                    
                case 2: // Test arm motor (6000+ RPM)
                    currentTest = "ARM MOTOR (6000+ RPM - Limited Power)";
                    instructions = "Arm should move up slowly with limited power";
                    robot.setArmPower(HIGH_SPEED_TEST);
                    break;
                    
                case 3: // Test intake motor (6000+ RPM)
                    currentTest = "INTAKE MOTOR (6000+ RPM - Limited Power)";
                    instructions = "Intake should spin forward with limited power";
                    robot.setIntakePower(HIGH_SPEED_TEST);
                    break;
                    
                case 4: // Test lift motor (6000+ RPM)
                    currentTest = "LIFT MOTOR (6000+ RPM - Limited Power)";
                    instructions = "Lift should move up slowly with limited power";
                    robot.setLiftPower(HIGH_SPEED_TEST);
                    break;
                    
                case 5: // Test servos
                    currentTest = "SERVO TEST";
                    instructions = "Servos should move to center position";
                    robot.setClawPosition(0.5);
                    robot.setWristPosition(0.5);
                    break;
            }
            
            // ================== INDIVIDUAL MOTOR CONTROLS ==================
            
            // Manual drive control for testing directions (312 RPM precision motors)
            if (gamepad1.dpad_up) {
                robot.setTankDrive(PRECISION_DRIVE_TEST, PRECISION_DRIVE_TEST);  // Forward
            } else if (gamepad1.dpad_down) {
                robot.setTankDrive(-PRECISION_DRIVE_TEST, -PRECISION_DRIVE_TEST); // Backward
            } else if (gamepad1.dpad_left) {
                robot.setTankDrive(-PRECISION_DRIVE_TEST, PRECISION_DRIVE_TEST);  // Turn left
            } else if (gamepad1.dpad_right) {
                robot.setTankDrive(PRECISION_DRIVE_TEST, -PRECISION_DRIVE_TEST);  // Turn right
            }
            
            // Manual mechanism controls (6000+ RPM motors with limited power)
            if (gamepad1.left_bumper) {
                robot.setArmPower(HIGH_SPEED_TEST);     // Arm up
            } else if (gamepad1.left_trigger > 0.1) {
                robot.setArmPower(-HIGH_SPEED_TEST);    // Arm down
            }
            
            if (gamepad1.right_bumper) {
                robot.setLiftPower(HIGH_SPEED_TEST);    // Lift up
            } else if (gamepad1.right_trigger > 0.1) {
                robot.setLiftPower(-HIGH_SPEED_TEST);   // Lift down
            }
            
            if (gamepad1.y) {
                robot.setIntakePower(HIGH_SPEED_TEST);  // Intake forward
            } else if (gamepad1.x) {
                robot.setIntakePower(-HIGH_SPEED_TEST); // Intake reverse
            }
            
            // ================== TELEMETRY ==================
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current Test", currentTest);
            telemetry.addData("Instructions", instructions);
            telemetry.addLine();
            
            telemetry.addData("Drive Motors (312 RPM)", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f", 
                            robot.leftFrontDrive.getPower(),
                            robot.rightFrontDrive.getPower(), 
                            robot.leftBackDrive.getPower(),
                            robot.rightBackDrive.getPower());
            
            telemetry.addData("Mechanism Motors (6000+ RPM)", "Arm:%.2f Intake:%.2f Lift:%.2f",
                            robot.armMotor.getPower(),
                            robot.intakeMotor.getPower(),
                            robot.liftMotor.getPower());
            
            telemetry.addData("Servos", "Claw:%.2f Wrist:%.2f", 
                            robot.clawServo.getPosition(),
                            robot.wristServo.getPosition());
            
            telemetry.addLine();
            telemetry.addData("Manual Controls", "DPad=Drive, Bumpers/Triggers=Mechanisms");
            telemetry.addData("Manual Controls", "Y/X=Intake, LB/LT=Arm, RB/RT=Lift");
            
            // Show encoder values for precision drive motors
            int[] encoders = robot.getDriveEncoders();
            telemetry.addData("Drive Encoders (312 RPM)", "LF:%d RF:%d LB:%d RB:%d", 
                            encoders[0], encoders[1], encoders[2], encoders[3]);
            
            telemetry.update();
        }
    }
}