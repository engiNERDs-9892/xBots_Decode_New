package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class Spindexer {
	private final static double DEGREES_TO_SERVO = (double) 1 / 355;
	private int selectedSegment = 0;
	private boolean isIntakePosition = true;
	private final static double[] INTAKE_DEGREE_POSITIONS = {0.0, 240.0, 120.0};
	private final static double[] TRANSFER_DEGREE_POSITIONS = {180.0, 60.0, 300.0};
	private boolean wasAPressedLast = false;


	private final Servo spindexer;
	public Spindexer(String spindexerName, HardwareMap hardwareMap) {
		spindexer = hardwareMap.get(Servo.class, spindexerName);
	}

	public void update(Gamepad gamepad) {
		if (gamepad.x) {
			selectedSegment = 0;
		}

		if (gamepad.y) {
			selectedSegment = 1;
		}

		if (gamepad.b) {
			selectedSegment = 2;
		}

		if (gamepad.a) {
			isIntakePosition = !isIntakePosition;
		}

		spindexer.setPosition(
			isIntakePosition ?
				INTAKE_DEGREE_POSITIONS[selectedSegment] * DEGREES_TO_SERVO :
				TRANSFER_DEGREE_POSITIONS[selectedSegment] * DEGREES_TO_SERVO
		);
	}

	public String getTelemetryData() {
		return String.format("selectedSegment: %d\nisIntakePosition: %b", selectedSegment, isIntakePosition);
	}
}