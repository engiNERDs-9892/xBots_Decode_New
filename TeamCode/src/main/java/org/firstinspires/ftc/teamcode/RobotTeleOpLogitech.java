package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp mode optimized for Logitech controller with goBILDA motors
 * - Low RPM motors (312 RPM) for precise drivetrain control
 * - High RPM motors (6000+ RPM) for fast mechanisms
 * - Comprehensive control mapping for Logitech F310/F710 controllers
 */
@TeleOp(name="Robot TeleOp - Logitech", group="Linear Opmode")
public class RobotTeleOpLogitech extends LinearOpMode {

    // Declare OpMode members
    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    
    // Control variables
    private double driveSpeed = 1.0;        // Normal drive speed (312 RPM motors can handle full power)
    private double precisionSpeed = 0.4;    // Precision drive speed
    private double armSpeed = 0.4;          // Arm movement speed (6000+ RPM motor - needs limiting)
    private double intakeSpeed = 0.8;       // Intake speed (6000+ RPM motor)
    private double liftSpeed = 0.6;         // Lift speed (6000+ RPM motor)
    
    // Servo positions
    private double clawOpenPosition = 0.0;
    private double clawClosedPosition = 1.0;
    private double wristUpPosition = 0.0;
    private double wristDownPosition = 1.0;
    
    // State tracking
    private boolean clawOpen = true;
    private boolean lastClawButton = false;
    private boolean lastWristButton = false;
    
    @Override
    public void runOpMode() {
        
        // Initialize the hardware variables
        robot.init(hardwareMap);
        
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Robot Ready - Logitech Controller");
        telemetry.addData("Drive Speed", "%.1f (Right Bumper = Precision)", driveSpeed);
        telemetry.addData("Drive Motors", "312 RPM (Precision Control)");
        telemetry.addData("Mechanism Motors", "6000+ RPM (High Speed)");
        telemetry.addData("Controls", "Left Stick = Drive, Right Stick = Turn/Strafe");
        telemetry.addData("Gamepad1", "A=Claw, B=Wrist, Y=Intake, X=Outtake");
        telemetry.addData("Gamepad1", "DPad Up/Down=Arm, Triggers=Lift");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // ================== DRIVETRAIN CONTROL ==================
            
            // Get gamepad inputs
            double drive = -gamepad1.left_stick_y;    // Forward/backward (inverted for intuitive control)
            double strafe = gamepad1.left_stick_x;    // Left/right strafe
            double twist = gamepad1.right_stick_x;    // Rotation
            
            // Apply deadzone to prevent drift
            drive = Math.abs(drive) > 0.1 ? drive : 0;
            strafe = Math.abs(strafe) > 0.1 ? strafe : 0;
            twist = Math.abs(twist) > 0.1 ? twist : 0;
            
            // Determine speed mode
            double currentDriveSpeed = gamepad1.right_bumper ? precisionSpeed : driveSpeed;
            
            // Apply mecanum drive
            robot.mecanumDrive(drive, strafe, twist, currentDriveSpeed);
            
            // ================== ARM CONTROL (6000+ RPM Motor) ==================
            
            double armPower = 0;
            if (gamepad1.dpad_up) {
                armPower = armSpeed;        // Arm up (limited power for high RPM motor)
            } else if (gamepad1.dpad_down) {
                armPower = -armSpeed;       // Arm down (limited power for high RPM motor)
            }
            robot.setArmPower(armPower);
            
            // ================== LIFT CONTROL (6000+ RPM Motor) ==================
            
            double liftPower = 0;
            if (gamepad1.right_trigger > 0.1) {
                liftPower = gamepad1.right_trigger * liftSpeed;    // Lift up
            } else if (gamepad1.left_trigger > 0.1) {
                liftPower = -gamepad1.left_trigger * liftSpeed;   // Lift down
            }
            robot.setLiftPower(liftPower);
            
            // ================== INTAKE CONTROL ==================
            
            double intake = 0;
            if (gamepad1.y) {
                intake = intakeSpeed;       // Intake in
            } else if (gamepad1.x) {
                intake = -intakeSpeed;      // Intake out (outtake)
            }
            robot.setIntakePower(intake);
            
            // ================== SERVO CONTROLS ==================
            
            // Claw control (A button - toggle)
            boolean currentClawButton = gamepad1.a;
            if (currentClawButton && !lastClawButton) {  // Button just pressed
                clawOpen = !clawOpen;
                robot.setClawPosition(clawOpen ? clawOpenPosition : clawClosedPosition);
            }
            lastClawButton = currentClawButton;
            
            // Wrist control (B button - toggle)
            boolean currentWristButton = gamepad1.b;
            if (currentWristButton && !lastWristButton) {  // Button just pressed
                robot.setWristPosition(robot.wristServo.getPosition() > 0.5 ? wristUpPosition : wristDownPosition);
            }
            lastWristButton = currentWristButton;
            
            // ================== ALTERNATIVE GAMEPAD2 CONTROLS ==================
            
            if (gamepad2.left_stick_y != 0) {
                robot.setArmPower(-gamepad2.left_stick_y * armSpeed);
            }
            
            if (gamepad2.right_stick_y != 0) {
                robot.setLiftPower(-gamepad2.right_stick_y * liftSpeed);
            }
            
            // ================== TELEMETRY ==================
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Mode", gamepad1.right_bumper ? "PRECISION" : "NORMAL");
            telemetry.addData("Motors", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f", 
                            robot.leftFrontDrive.getPower(),
                            robot.rightFrontDrive.getPower(), 
                            robot.leftBackDrive.getPower(),
                            robot.rightBackDrive.getPower());
            telemetry.addData("Arm Power", "%.2f", robot.armMotor.getPower());
            telemetry.addData("Lift Power", "%.2f", robot.liftMotor.getPower());
            telemetry.addData("Intake Power", "%.2f", robot.intakeMotor.getPower());
            telemetry.addData("Claw", clawOpen ? "OPEN" : "CLOSED");
            telemetry.addData("Servos", "Claw:%.2f Wrist:%.2f", 
                            robot.clawServo.getPosition(),
                            robot.wristServo.getPosition());
            
            // Show encoder values for debugging
            int[] encoders = robot.getDriveEncoders();
            telemetry.addData("Encoders", "LF:%d RF:%d LB:%d RB:%d", 
                            encoders[0], encoders[1], encoders[2], encoders[3]);
            
            telemetry.update();
        }
    }
}