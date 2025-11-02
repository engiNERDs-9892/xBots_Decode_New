package org.firstinspires.ftc.teamcode.stellarstructure.runnables;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.stellarstructure.hardwaremapwrappers.StellarDcMotor;
import org.firstinspires.ftc.teamcode.stellarstructure.Subsystem;

public class MoveTo extends Directive {
	private final StellarDcMotor motor;
	private final int targetPosition;
	private final double power;
	private final double acceptableRange;

	public MoveTo(StellarDcMotor motor, int targetPosition, double power) {
		this(motor, targetPosition, power, 5);
	}

	public MoveTo(StellarDcMotor motor, int targetPosition, double power, double acceptableRange) {
		this.motor = motor;
		this.targetPosition = targetPosition;
		this.power = power;
		this.acceptableRange = acceptableRange;
		setInterruptible(true);
	}

	@Override
	public void start(boolean hadToInterruptToStart) {
		motor.setTargetPosition(targetPosition);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(Math.abs(power));
	}

	@Override
	public void update() {}

	@Override
	public void stop(boolean interrupted) {
		motor.setPower(0);
	}

	public MoveTo requires(Subsystem... subsystems) {
		setRequiredSubsystems(subsystems);
		return this;
	}

	public MoveTo interruptible(boolean interruptible) {
		setInterruptible(interruptible);
		return this;
	}

	@Override
	public boolean isFinished() {
		return Math.abs(motor.getCurrentPosition() - targetPosition) <= acceptableRange;
	}
}
