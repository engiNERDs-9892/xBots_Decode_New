package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.gamepad.Gamepad;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import org.firstinspires.ftc.teamcode.util.LoggingConfig;

@Config
@TeleOp
public class TeleopNew extends CommandOpMode {
    private static final String TAG = "TeleopNew";


    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private GamepadEx gamepad;
    private Intake intake;
    private Shooter shooter;
    public static double shooterX, shooterY;

    // Performance monitoring
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private int loopCount = 0;
    private long lastLogTime = 0;

    @Override
    public void initialize() {
        RobotLog.ii(TAG, "=== TeleopNew initialization started ===");
        long startTime = System.currentTimeMillis();

        try {
            // Initialize follower
            RobotLog.ii(TAG, "Creating Pedro Pathing follower...");
            follower = Constants.createFollower(hardwareMap);
            RobotLog.ii(TAG, "Follower created successfully");

            RobotLog.ii(TAG, "Setting starting pose from Memory: x=%.2f, y=%.2f, heading=%.2f°",
                Memory.robotPose.getX(), Memory.robotPose.getY(), Math.toDegrees(Memory.robotPose.getHeading()));
            follower.setStartingPose(Memory.robotPose);

            super.reset();
            RobotLog.ii(TAG, "Command scheduler reset completed");

            follower.startTeleopDrive();
            RobotLog.ii(TAG, "TeleOp drive mode started");

            gamepad = new GamepadEx(gamepad1);
            RobotLog.ii(TAG, "GamepadEx initialized");

            intake = new Intake(hardwareMap);
            RobotLog.ii(TAG, "Intake subsystem initialized");

            // Configure shooter position based on alliance
            if (Memory.allianceRed) {
                shooterX = 144;
                shooterY = 144;
                RobotLog.ii(TAG, "Red alliance detected - Shooter target: (%.1f, %.1f)", shooterX, shooterY);
            } else {
                shooterX = 0;
                shooterY = 144;
                RobotLog.ii(TAG, "Blue alliance detected - Shooter target: (%.1f, %.1f)", shooterX, shooterY);
            }

            shooter = new Shooter(hardwareMap, () -> follower.getPose(), shooterX, shooterY);
            RobotLog.ii(TAG, "Shooter subsystem initialized");

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error during initialization: %s", e.getMessage());
            RobotLog.ee(TAG, "Stack trace: %s", android.util.Log.getStackTraceString(e));
            throw e;
        }

        // Configure gamepad button mappings with logging
        RobotLog.ii(TAG, "Configuring gamepad button mappings...");

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                intake.collect()
        );
        RobotLog.dd(TAG, "Right bumper mapped to intake collect");

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                intake.reverse()
        );
        RobotLog.dd(TAG, "DPAD down mapped to intake reverse");

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                intake.stop()
        );
        RobotLog.dd(TAG, "Right bumper release mapped to intake stop");

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ParallelCommandGroup(
                    intake.collect(),
                    intake.open()
                )
        );
        RobotLog.dd(TAG, "Left bumper press mapped to intake collect + open");

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                new ParallelCommandGroup(
                        intake.stop(),
                        intake.close()
                )
        );
        RobotLog.dd(TAG, "Left bumper release mapped to intake stop + close");

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                shooter.flywheel(true)
        );
        RobotLog.dd(TAG, "Button A mapped to shooter flywheel ON");

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                shooter.flywheel(false)
        );
        RobotLog.dd(TAG, "Button B mapped to shooter flywheel OFF");

        long elapsedTime = System.currentTimeMillis() - startTime;
        RobotLog.ii(TAG, "=== TeleopNew initialization completed successfully in %dms ===", elapsedTime);

        // Start performance monitoring
        runtime.reset();
        RobotLog.ii(TAG, "Performance monitoring started");
    }

    @Override
    public void run() {
        loopTimer.reset();
        loopCount++;

        try {
            super.run();

            // Log gamepad inputs periodically (every 2 seconds)
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastLogTime > 2000) {
                logGamepadInputs();
                logRobotState();
                lastLogTime = currentTime;
            }

            // Drive control with input logging
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // Log significant stick movements
            if (LoggingConfig.ENABLE_DEBUG_LOGGING && (Math.abs(drive) > 0.1 || Math.abs(strafe) > 0.1 || Math.abs(turn) > 0.1)) {
                RobotLog.dd(TAG, "Drive inputs - Drive: %.2f, Strafe: %.2f, Turn: %.2f", drive, strafe, turn);
            }

            follower.setTeleOpDrive(drive, strafe, turn, true);
            follower.update();

            telemetryData.addData("X", follower.getPose().getX());
            telemetryData.addData("Y", follower.getPose().getY());
            telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetryData.update();

            // Performance monitoring every 100 loops (~2 seconds at 50Hz)
            if (LoggingConfig.ENABLE_PERFORMANCE_LOGGING && loopCount % 100 == 0) {
                long loopTime = (long) loopTimer.milliseconds();
                logPerformanceMetrics(loopTime);
            }

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error in run loop %d: %s", loopCount, e.getMessage());
            RobotLog.ee(TAG, "Stack trace: %s", android.util.Log.getStackTraceString(e));
        }
    }

    /**
     * Log current gamepad inputs for debugging
     */
    private void logGamepadInputs() {
        RobotLog.ii(TAG, "=== Gamepad Status ===");
        RobotLog.ii(TAG, "Left stick: (%.2f, %.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
        RobotLog.ii(TAG, "Right stick: (%.2f, %.2f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
        RobotLog.ii(TAG, "Triggers: L=%.2f, R=%.2f", gamepad1.left_trigger, gamepad1.right_trigger);

        // Log active buttons
        StringBuilder activeButtons = new StringBuilder();
        if (gamepad1.a) activeButtons.append("A ");
        if (gamepad1.b) activeButtons.append("B ");
        if (gamepad1.x) activeButtons.append("X ");
        if (gamepad1.y) activeButtons.append("Y ");
        if (gamepad1.left_bumper) activeButtons.append("LB ");
        if (gamepad1.right_bumper) activeButtons.append("RB ");
        if (gamepad1.dpad_up) activeButtons.append("DU ");
        if (gamepad1.dpad_down) activeButtons.append("DD ");
        if (gamepad1.dpad_left) activeButtons.append("DL ");
        if (gamepad1.dpad_right) activeButtons.append("DR ");

        if (activeButtons.length() > 0) {
            RobotLog.ii(TAG, "Active buttons: %s", activeButtons.toString().trim());
        }
    }

    /**
     * Log current robot state for debugging
     */
    private void logRobotState() {
        if (follower != null) {
            RobotLog.ii(TAG, "=== Robot State ===");
            RobotLog.ii(TAG, "Position: (%.2f, %.2f)", follower.getPose().getX(), follower.getPose().getY());
            RobotLog.ii(TAG, "Heading: %.2f degrees", Math.toDegrees(follower.getPose().getHeading()));
            RobotLog.ii(TAG, "Runtime: %.2f seconds", runtime.seconds());
            RobotLog.ii(TAG, "Loop count: %d", loopCount);
        }
    }

    /**
     * Log performance metrics
     */
    private void logPerformanceMetrics(long loopTime) {
        if (loopTime > 20) {
            RobotLog.ww(TAG, "PERFORMANCE WARNING: Loop time %dms (target <20ms)", loopTime);
        } else {
            RobotLog.dd(TAG, "Performance: Loop %d completed in %dms", loopCount, loopTime);
        }

        double avgLoopTime = (runtime.milliseconds() * 1000) / loopCount;
        RobotLog.ii(TAG, "Average loop time: %.2fms over %d loops", avgLoopTime, loopCount);
    }
}