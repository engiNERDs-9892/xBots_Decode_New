package org.firstinspires.ftc.teamcode.Decode_2025;// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tank TeleOp", group = "Decode")
public class _TankTeleOpDecode extends LinearOpMode {
  DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
  double gpLy = 0.0;
  double gpRy = 0.0;
  double gpLx = 0.0;
  double gpRx = 0.0;
  double gpLt = 0.0;
  double gpRt = 0.0;

  @Override
  public void runOpMode() {
    try {
      telemetry.addLine("A - 90 deg STRAF");
      telemetry.addLine("left stick Y left wheel");
      telemetry.addLine("right stick Y right wheel");
      telemetry.addLine("B  button  toggle");
      telemetry.addLine(" speed 70% or 100%");
      waitForStart();
      while (opModeIsActive()) {
        if (gamepad1.a) {
        }

        if (gamepad1.b) {
        }
        gpLt = gamepad1.left_trigger;
        gpLy = gamepad1.left_stick_y;
        gpRt = gamepad1.right_trigger;
        gpRy = gamepad1.right_stick_y;
        // scaleRange(min, max) configure the following servo control by trigger
        // to straf drive to finish alignment to shoot: by April tag
        // check angle of wheel if not 180 then
        gpLt = (1.0 - gamepad1.left_trigger)/2.0;
        gpRt = 0.5 - (gamepad1.right_trigger)/2.0;
        if (gpLt < 0.5){
          // turn left by servo
          drive.lfTurn.setPosition(gpLt);
          drive.rtTurn.setPosition(0.5);
        } else {
          // wheels lock 180/0
          drive.lfTurn.setPosition(0.5);
          drive.rtTurn.setPosition(0.5);
        }
        if (gpRt > 0.5){
          // turn right by servo
          drive.lfTurn.setPosition(0.5);
          drive.rtTurn.setPosition(gpRt);
        } else {
          // wheels lock 180/0
          drive.lfTurn.setPosition(0.5);
          drive.rtTurn.setPosition(0.5);
        }
        // determine if 90 degrees then turn and lock left
        // determine if -90 degrees then turn and lock right
        //drive wheels
        drive.lfDrive.setPower(gpLy);
        drive.rtDrive.setPower(gpRy);
      }// while op Mode
    } catch (Exception e) {
      telemetry.addLine(", exception in gamePadTeleOP");
      telemetry.update();
      sleep(2000);
      requestOpModeStop();
    }//catch
  }// run Op mode
}// tank TeleOp
