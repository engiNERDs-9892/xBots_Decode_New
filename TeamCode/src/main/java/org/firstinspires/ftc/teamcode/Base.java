package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LeverTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Base TARS -JetEngine", group = "Robot")
public class Base extends LinearOpMode {
	@Override
	public void runOpMode() {
		Drivebase drivebase = new Drivebase(
				"leftFront",
				"leftBack",
				"rightFront",
				"rightBack",
				1.00, 1.00,
				hardwareMap
		);

		Intake intake = new Intake(
				"intake",
				hardwareMap
		);

		LeverTransfer leverTransfer = new LeverTransfer(
				"leverTransfer",
				0.00, 0.20,
				hardwareMap
		);

		Spindexer spindexer = new Spindexer(
				"spindexer",
				hardwareMap
		);

		//todo: lever neutral: 0.3, lever up: 0.0
		//todo: make subsystems class have single click feature

		waitForStart();

		if (isStopRequested()) return;

		while (opModeIsActive()) {
			drivebase.update(gamepad1);
			intake.update(gamepad1);
			leverTransfer.update(gamepad1);
			spindexer.update(gamepad1);


			telemetry.addLine(drivebase.getTelemetryData());
			telemetry.addLine(intake.getTelemetryData());
			telemetry.addLine(leverTransfer.getTelemetryData());
			telemetry.addLine(spindexer.getTelemetryData());
			telemetry.addLine("Honesty Setting: 90%");

			telemetry.update();
		}
	}
}