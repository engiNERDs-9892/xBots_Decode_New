package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


//@Disabled
@TeleOp
//@Disabled
public class xBots_Driver extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Declare and Create Hardware Map for Motors, Servos and Sensors

        //Motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");
        DcMotor flyWheel = hardwareMap.dcMotor.get("flyWheel");

        //Servos
        Servo flipper = hardwareMap.servo.get("flipper");


        //Sensors
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        //Set Motor Directions on Baby Bot the Left Side is Reversed
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /////////////////////////////////////////////////////////////////////////////////
        //////////////        ADJUST HUB DIRECTIONS TO SET HEADINGS        //////////////
        /////////////////////////////////////////////////////////////////////////////////
        //Ignore comment above
        //Copied from gobilda sample code please adjust values

        //odo.setOffsets(-84.0, -168.0);
        //odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


        //Set Initial Servo Posiiton like below
        flipper.setPosition(0.48);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){

            /////////////////////////DRIVE CODE///////////////////////////////////////
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double botHeading = odo.getHeading(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // This gives the motor power to the wheels relative to the placement of the left and right joystick
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            //Update the heading given by the pinpoint odometry controller
            odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

            //frontLeftMotor.setPower(frontLeftPower);
            //backLeftMotor.setPower(backLeftPower);
            //frontRightMotor.setPower(frontRightPower);
            //backRightMotor.setPower(backRightPower);


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            if (gamepad1.back) {
                odo.resetPosAndIMU();
            }

            //Use Super Slow Mode if Left Trigger is pushed
            if (gamepad1.left_trigger != 0) {
                frontLeftMotor.setPower(.18 * frontLeftPower);
                backLeftMotor.setPower(.18 * backLeftPower);
                frontRightMotor.setPower(.18 * frontRightPower);
                backRightMotor.setPower(.18 * backRightPower);
            }



            //Run at Full Power if not button is pushed
            else {
                frontLeftMotor.setPower(.9 * frontLeftPower);
                backLeftMotor.setPower(.9 * backLeftPower);
                frontRightMotor.setPower(.9 * frontRightPower);
                backRightMotor.setPower(.9 * backRightPower);
            }

            telemetry.addData("botHeading", "%f", botHeading);
            telemetry.update();

            /////////////Move Flipper///////////////////////
            //To raise flipper
            if (gamepad2.aWasPressed()) {
                flipper.setPosition(.33);
            }
            //To Open the Claws Push y
            else if (gamepad2.aWasReleased()) {
                flipper.setPosition(.48);
            }

            /////////////Hold Right Trigger to Use Fly Wheel///////////////////////
            if (gamepad2.right_trigger != 0){
                flyWheel.setPower(.65);
            } else if (gamepad2.right_trigger == 0) {;
                flyWheel.setPower(0);
            }

        }//OpModeIsActive
    }//RunOpMode
}//LinearOpMode
