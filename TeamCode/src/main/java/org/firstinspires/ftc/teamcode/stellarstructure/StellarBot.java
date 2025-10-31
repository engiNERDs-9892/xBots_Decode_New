package org.firstinspires.ftc.teamcode.stellarstructure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StellarBot {
	protected final Subsystem[] subsystems;

	public StellarBot(Subsystem... subsystems) {
		this.subsystems = subsystems;

		//add subsystems to scheduler
		for (Subsystem subsystem : subsystems) {
			Scheduler.getInstance().addSubsystem(subsystem);
		}
	}

	public void init(HardwareMap hardwareMap) {
		//initialize all subsystems
		for (Subsystem subsystem : subsystems) {
			subsystem.init(hardwareMap);
		}
	}

	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		//set gamepads for all subsystems
		for (Subsystem subsystem : subsystems) {
			subsystem.setGamepads(gamepad1, gamepad2);
		}
	}

	public void update() {
		//update triggers and directives
		Scheduler.getInstance().run();

		//update subsystems
		for (Subsystem subsystem : subsystems) {
			subsystem.update();
		}
	}

	public String getTelemetryData() {
		StringBuilder telemetry = new StringBuilder();

		for (Subsystem subsystem: subsystems) {
			telemetry.append(subsystem.getTelemetryData()).append('\n');
		}

		telemetry.append(Scheduler.getInstance().getTelemetry());

		return telemetry.toString();
	}

	public void cancelAll() {
		Scheduler.getInstance().cancelAll();
	}
}