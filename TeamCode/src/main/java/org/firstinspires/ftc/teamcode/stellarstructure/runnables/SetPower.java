package org.firstinspires.ftc.teamcode.stellarstructure.runnables;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.stellarstructure.Subsystem;
import org.firstinspires.ftc.teamcode.stellarstructure.hardwaremapwrappers.StellarDcMotor;

public class SetPower extends Directive {
	private final StellarDcMotor motor;
	private final double power;

	public SetPower(StellarDcMotor motor, double power) {
		this.motor = motor;
		this.power = power;
		setInterruptible(true);
	}

	@Override
	public void start(boolean hadToInterruptToStart) {
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor.setPower(power);
	}

	@Override
	public void update() {}

	@Override
	public void stop(boolean interrupted) {

	}

	public SetPower requires(Subsystem... subsystems) {
		setRequiredSubsystems(subsystems);
		return this;
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}