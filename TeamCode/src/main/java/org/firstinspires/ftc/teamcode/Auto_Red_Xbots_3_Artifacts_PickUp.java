package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class Auto_Red_Xbots_3_Artifacts_PickUp extends LinearOpMode {

    final int inches = 42;//converts ticks to inches traveled (537.7 ticks/rev * 1Rev/104pi mm * 1mm/0.039 in) = 42.198 ticks/inch


    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;
    DcMotor flyWheel = null;
    DcMotor InTake = null;
    Servo flipper;





    @Override
    public void runOpMode() throws InterruptedException {
        //Declare and Create Hardware Map for Motors, Servos and Sensors

        //Motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        flyWheel = hardwareMap.dcMotor.get("flyWheel");
        flipper = hardwareMap.servo.get("flipper");
        InTake = hardwareMap.dcMotor.get("InTake");
        //Servos


        //Tell Launch motor to run using encoders
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        InTake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //Set motors and servos to brake for initialization
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipper.setPosition(.50);
        InTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        flyWheel.setPower(.48);
        Move(directions.FORWARDS,24,.70);
        sleep (2000);
        flipper.setPosition(.33);// Raiser Flipper for 1st Artifact
        sleep (1000);
        flipper.setPosition(.50);//Lower Flipper
        sleep(3000);
        flipper.setPosition(.33);// Raiser Flipper for 2nd Artifact
        sleep (1000);
        flipper.setPosition(.50);//Lower Flipper
        sleep(3000);
        flipper.setPosition(.33);// Raiser Flipper for 3rd Artifact
        sleep(1000);
        flipper.setPosition(.50);
        sleep(500);
        Move(directions.FORWARDS,5,.50);//can speed up
        Move(directions.COUNTERCLOCKWISE,34,.60);//dont speed up.
        Move(directions.RIGHT,30,.67);//s

        flyWheel.setPower(0);
        InTake.setPower(1.00);
        sleep(500);
        Move(directions.FORWARDS,40,.20);
        sleep(1500);
        Move(directions.BACKWARDS,30,75);
        InTake.setPower(0);
        flyWheel.setPower(.48);
        sleep(500);
        Move(directions.LEFT,35,85);
        Move(directions.CLOCKWISE,55,.35);
        sleep (1000);
        flipper.setPosition(.33);// Raiser Flipper for 1st Artifact
        sleep (1000);
        flipper.setPosition(.50);//Lower Flipper
        sleep(2000);
        flipper.setPosition(.33);// Raiser Flipper for 2nd Artifact
        sleep (1000);
        flipper.setPosition(.50);//Lower Flipper
        sleep(2000);
        flipper.setPosition(.33);// Raiser Flipper for 3rd Artifact
        sleep(1000);
        flipper.setPosition(.50);
        sleep(500);
        Move(directions.COUNTERCLOCKWISE,34,.35);
        Move(directions.RIGHT,20,.67);
        flyWheel.setPower(0);
        Move(directions.RIGHT,10,.30);




    }//RunOpMode

    //////////////Move Function////////////
    private void Move (directions directions, int target, double speed){
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Sets the motor directions based on the directions parameter
        if (directions == Auto_Red_Xbots_3_Artifacts_PickUp.directions.FORWARDS){
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if (directions == Auto_Red_Xbots_3_Artifacts_PickUp.directions.BACKWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (directions == Auto_Red_Xbots_3_Artifacts_PickUp.directions.LEFT) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        else if (directions == Auto_Red_Xbots_3_Artifacts_PickUp.directions.RIGHT) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        else if (directions == Auto_Red_Xbots_3_Artifacts_PickUp.directions.CLOCKWISE) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        else if (directions == Auto_Red_Xbots_3_Artifacts_PickUp.directions.COUNTERCLOCKWISE) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Gives it a position to run to
        motorFL.setTargetPosition(target * inches);
        motorFR.setTargetPosition(target * inches);
        motorBL.setTargetPosition(target * inches);
        motorBR.setTargetPosition(target * inches);


        // tells it to go to the position that is set
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // the motor speed for Wheels
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);




        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && ((motorFL.isBusy() || motorFR.isBusy()))) {
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }//End Move Function

    /////////////////Directions List////////
    enum directions{
       FORWARDS,
        BACKWARDS,
        LEFT,
        RIGHT,
        CLOCKWISE,
        COUNTERCLOCKWISE,
   }//Directions

}//LinearOpMode
