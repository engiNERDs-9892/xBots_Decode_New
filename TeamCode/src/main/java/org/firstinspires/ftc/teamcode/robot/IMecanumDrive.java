package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.TelemetryMirror;

public interface IMecanumDrive {
    public void run(Gamepad driveGamepad, TelemetryMirror telemetry);

    public void stop(TelemetryMirror telemetry);
    public void hold();

    public DcMotor getFrontLeft();
    public DcMotor getFrontRight();
    public DcMotor getBackLeft();
    public DcMotor getBackRight();
}
