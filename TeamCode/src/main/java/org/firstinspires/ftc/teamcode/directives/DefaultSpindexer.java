package org.firstinspires.ftc.teamcode.directives;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stellarstructure.Trigger;
import org.firstinspires.ftc.teamcode.stellarstructure.conditions.GamepadButton;
import org.firstinspires.ftc.teamcode.stellarstructure.conditions.StatefulCondition;
import org.firstinspires.ftc.teamcode.stellarstructure.runnables.DefaultDirective;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class DefaultSpindexer extends DefaultDirective {
	//todo: implement starting conditions and directives and procedures
	public DefaultSpindexer(Spindexer spindexer, Gamepad gamepad1) {
		super(spindexer);

		addTrigger(new Trigger(
			new GamepadButton(gamepad1, GamepadButton.Button.X), //when X held
			() -> {
				spindexer.setSelectedSegment(0);
				spindexer.updateServoPosition();
			}
		));

		addTrigger(new Trigger(
				new GamepadButton(gamepad1, GamepadButton.Button.Y), //when Y held
				() -> {
					spindexer.setSelectedSegment(1);
					spindexer.updateServoPosition();
				}
		));

		addTrigger(new Trigger(
				new GamepadButton(gamepad1, GamepadButton.Button.B), //when B held
				() -> {
					spindexer.setSelectedSegment(2);
					spindexer.updateServoPosition();
				}
		));

		addTrigger(new Trigger(
				new StatefulCondition(
						new GamepadButton(gamepad1, GamepadButton.Button.A),
						StatefulCondition.Edge.RISING //on initial press
				),
				() -> {
					spindexer.toggleIsIntakePosition();
					spindexer.updateServoPosition();
				}
		));
	}
}
