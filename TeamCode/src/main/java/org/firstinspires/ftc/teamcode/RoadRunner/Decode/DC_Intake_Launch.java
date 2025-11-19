package org.firstinspires.ftc.teamcode.RoadRunner.Decode;
// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

public class DC_Intake_Launch {
  /* Declare OpMode members.
   * gain access to methods in the calling OpMode.
   */
  private LinearOpMode myOp = null;

  // Default constructor
  public DC_Intake_Launch(LinearOpMode opmode) {
    myOp = opmode;
      present = new DC_BallSensor(myOp);
  }

  public DC_BallSensor present;

  private DcMotorEx launch = null; // 6000 rpm motor
  private DcMotor arm = null; // 312 rpm motor
  private Servo intake = null; // intake motor controller
  private Servo guide = null; // intake motor controller
  //private Servo gate = null; // intake motor controller
  private AnalogInput armpot = null; // Arm Potentiometer input ( servo pot)

  // status light
  private final RevBlinkinLedDriver status = null;
  private final RevBlinkinLedDriver.BlinkinPattern pattern = null;
  private Deadline ledCycleDeadline;

  DisplayKind displayKind;

  protected enum DisplayKind {
    MANUAL,
    AUTO
  }

  public void InitIL() {

    // Define and Initialize Motor.
    launch = myOp.hardwareMap.get(DcMotorEx.class, "launch");
    launch.setDirection(DcMotorSimple.Direction.FORWARD); // todo set launch direction
    launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // prepare use velocity
    arm = myOp.hardwareMap.get(DcMotor.class, "arm");
    arm.setDirection(DcMotorSimple.Direction.FORWARD); // todo set arm direction
    // Define and Initialize Servo
    intake = myOp.hardwareMap.get(Servo.class, "intake");
    guide = myOp.hardwareMap.get(Servo.class, "guide");
    //gate = myOp.hardwareMap.get(Servo.class, "gate");
    armpot = myOp.hardwareMap.get(AnalogInput.class, "armpot");
    present.SensorInit();
    //status = myOp.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    //pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    //status.setPattern(pattern);
  }

  // Servo controlled motor
  public void Intake(double FS) {
    intake.setPosition(FS); // front intake servo motor
    // adjust polarity to set proper direction  full 0 or 1
    // may need to check .5 to stop intake
    guide.setPosition(FS); // guide intake
  }

  /* ball gate
  public void setGate(double FS) {
    gate.setPosition(FS);
  }*/
  // the following are motor checks only
  public void startLaunch(){
    launch.setPower(1.0);
  }
  public void stopLaunch(){
    launch.setPower(0.0);
  }
  public void startArm(){
    arm.setPower(1.0);
  }
  public void stopArm(){
    arm.setPower(0.0);
  }
  public void launchVelocity(){
    launch.setVelocity(1000.0);
    myOp.sleep(2000);
    myOp.telemetry.addData("Current Velocity", launch.getVelocity());
    myOp.telemetry.update();
    launch.setVelocity(0.0);
  }
  // end motor checks

  public boolean spinUp(double shoot_velocity) {
    boolean ready = true; // the not of false
    launch.setVelocity(shoot_velocity);
    while (myOp.opModeIsActive() && ready) {
      myOp.telemetry.addData("Current Velocity", launch.getVelocity());
      myOp.telemetry.update();
      if (launch.getVelocity() > shoot_velocity) ready = false;
    }
    // set status light
    return ready;
  } // end spin up

  public void spinOff() {
    launch.setVelocity(0.0);
  }

  public double getArmPot(){
    return armpot.getVoltage();
  }

  public boolean armPosition(int pos) {
    double armPwr = 0.3;
    double potVolt = 0.0;
    // analog voltage
    if (pos < 0) return false;
    if (pos > 2) return false;
    double home = .2;
    double launch = .5;
    double endGame = .7;
    double move = 0.0;
    if (pos == 0) move = home;
    else if (pos == 1) move = launch;
    else if (pos == 2) move = endGame;
    potVolt = armpot.getVoltage();
    if(potVolt < move) armPwr -= armPwr;
    arm.setPower(armPwr);
    while (myOp.opModeIsActive() && move != potVolt) {
     myOp.idle();
    }
    arm.setPower(0.0);
    return true;
  } // end arm position
}
