package org.firstinspires.ftc.teamcode.Decode_2025;// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_LaunchArm_Test extends LinearOpMode {
  private final DC_Intake_Launch game = new DC_Intake_Launch(this);
  @Override
  public void runOpMode() {

    game.InitIL();
    double armVolt = 0.0;
    boolean leftBump = false;
    boolean rightBump = false;
    int armPos = 0;

    waitForStart();

    while (opModeIsActive()) {
      armVolt = 0.0;
      for (int i = 0; i < 3; i++) {
        armVolt += game.getArmPot();
      }
      armVolt /= 3.0;
      telemetry.addData("Position Arm voltage",armVolt);
      telemetry.update();
      if(gamepad1.leftBumperWasPressed()) {
        leftBump = !leftBump;
        telemetry.addData("Activate Arm Postion:",leftBump);
      }
      if(gamepad1.rightBumperWasPressed()) {
        armPos += 1;
        if(armPos > 2) armPos = 0;
        telemetry.addData("         Arm Postion:",armPos);
        game.armPosition(armPos);
        sleep(1000);
      }
      if(leftBump){
        game.armPosition(armPos);
        sleep(1000);
      }
    }
  }
}
