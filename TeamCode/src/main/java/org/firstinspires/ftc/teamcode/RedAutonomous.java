package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;



@Autonomous
public class RedAutonomous extends LinearOpMode {

    //Servo intakeTilt = null;
    //Servo intake = null;
    //Servo xfer = null;
    //Servo leftIntake = null;
    //Servo rightIntake = null;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        //intakeTilt.setPosition(0.75); // based on levi's code
        //intake.setPosition(0);
        //rightIntake.setPosition(0.06);
        //leftIntake.setPosition(0.94);
        //xfer.setPosition(0.15);

        drive.followTrajectory(traj1);

        drive.followTrajectory(traj2);

    }
}