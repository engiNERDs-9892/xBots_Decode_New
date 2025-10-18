package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


public final class Intake {
	private final DcMotorEx intake;
	private double intakeSpeed = 0;

	public Intake(String intakeName, HardwareMap hardwareMap) {
		intake = hardwareMap.get(DcMotorEx.class, intakeName);
	}

	public void update(Gamepad gamepad) {
		if (gamepad.right_trigger > 0.0) {
			intakeSpeed = gamepad.right_trigger;
		} else {
			intakeSpeed = -gamepad.left_trigger;
		}

		intake.setPower(intakeSpeed);
	}

	public String getTelemetryData() {
		return String.format("Intake Speed: %f", intakeSpeed);
	}
}