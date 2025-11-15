package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.robot.Memory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.LoggingConfig;

@Config
@TeleOp
public class Teleop extends OpMode {
    private static final String TAG = "Teleop";

    // Logging control - managed via centralized LoggingConfig class
    // To modify logging behavior, edit LoggingConfig.java or use preset modes:
    // - LoggingConfig.setDevelopmentMode() - Maximum logging
    // - LoggingConfig.setCompetitionMode() - Minimal logging
    // - LoggingConfig.setTestingMode() - Balanced logging

    public static Follower follower;
    private PIDController controller, controllerTurret;
    private TelemetryManager telemetryM;
    public static double p = 0.2, i = 0.05, d = 0;
    public static double pT = 0.1, iT = 0, dT = 0;
    public static double f = 0.0265;
    private static double vel = 0;
    public static double target = 0;
    public static double alpha = 0.6;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    private DcMotorEx shooterb, shootert, intake, turret;
    private Servo hood;
    private VoltageSensor volt;
    public static double tangle = 40;
    public static double theta = 0;
    public static double shooterX = 135;
    public static double shooterY = 135;
    Servo latch;
    private double turretOffset = 0;
    private static final int TICKS_MIN = -330;
    private static final int TICKS_MAX = 990;
    public static  double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;
    public boolean stopAutoTurret = false;

    // Logging and monitoring
    private ElapsedTime teleopTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private int loopCount = 0;
    private long lastStatusLog = 0;
    @Override
    public void init() {
        RobotLog.ii(TAG, "=== Teleop initialization started ===");
        long initStartTime = System.currentTimeMillis();

        try {
            // Initialize PID controllers
            RobotLog.ii(TAG, "Initializing PID controllers...");
            controller = new PIDController(p, i, d);
            controllerTurret = new PIDController(pT, iT, dT);
            RobotLog.ii(TAG, "PID Controllers - Shooter: p=%.3f, i=%.3f, d=%.3f", p, i, d);
            RobotLog.ii(TAG, "PID Controllers - Turret: p=%.3f, i=%.3f, d=%.3f", pT, iT, dT);
            RobotLog.ii(TAG, "TICKS_PER_DEGREES: %.4f", TICKS_PER_DEGREES);

            // Initialize motors
            RobotLog.ii(TAG, "Initializing motors...");
            shooterb = hardwareMap.get(DcMotorEx.class, "sb");
            RobotLog.ii(TAG, "Shooter bottom motor initialized");

            turret = hardwareMap.get(DcMotorEx.class, "turret");
            int initialTurretPos = turret.getCurrentPosition();
            RobotLog.ii(TAG, "Turret motor initialized - Initial position: %d ticks", initialTurretPos);

            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RobotLog.ii(TAG, "Turret encoder reset");

            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotLog.ii(TAG, "Turret set to RUN_WITHOUT_ENCODER mode");

            shootert = hardwareMap.get(DcMotorEx.class, "st");
            RobotLog.ii(TAG, "Shooter top motor initialized");

            intake = hardwareMap.get(DcMotorEx.class, "intake");
            RobotLog.ii(TAG, "Intake motor initialized");

            // Initialize servos
            hood = hardwareMap.get(Servo.class, "hood");
            RobotLog.ii(TAG, "Hood servo initialized");

            latch = hardwareMap.servo.get("latch");
            RobotLog.ii(TAG, "Latch servo initialized");

            // Initialize voltage sensor
            volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
            double initialVoltage = volt.getVoltage();
            RobotLog.ii(TAG, "Voltage sensor initialized - Current: %.2fV", initialVoltage);

            if (initialVoltage < 11.5) {
                RobotLog.ww(TAG, "WARNING: Low battery voltage detected: %.2fV", initialVoltage);
            }

            // Build lookup tables
            RobotLog.ii(TAG, "Building RPM lookup table...");
            RPM.add(0, 330);
            RPM.add(39, 330);
            RPM.add(50, 355);
            RPM.add(60, 380);
            RPM.add(74, 395);
            RPM.add(90, 430);
            RPM.add(180, 430);
            RPM.createLUT();
            RobotLog.ii(TAG, "RPM LUT created with 7 data points");

            RobotLog.ii(TAG, "Building hood angle lookup table...");
            angle.add(0, 1);
            angle.add(20, 1);
            angle.add(39, 0.7);
            angle.add(50, 0.6);
            angle.add(60, 0.5);
            angle.add(74, 0.4);
            angle.add(90, 0.3);
            angle.add(180, 0.3);
            angle.createLUT();
            RobotLog.ii(TAG, "Hood angle LUT created with 8 data points");

            // Test LUTs
            double testRPM = RPM.get(60.0);
            double testAngle = angle.get(60.0);
            RobotLog.ii(TAG, "LUT test - Distance 60: RPM=%.1f, Angle=%.3f", testRPM, testAngle);

            long elapsedTime = System.currentTimeMillis() - initStartTime;
            RobotLog.ii(TAG, "=== Teleop initialization completed in %dms ===", elapsedTime);

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error during teleop initialization: %s", e.getMessage());
            RobotLog.ee(TAG, "Stack trace: %s", android.util.Log.getStackTraceString(e));
            throw e;
        }
    }

    @Override
    public void start() {
        RobotLog.ii(TAG, "=== Teleop start sequence initiated ===");

        try {
            follower = Constants.createFollower(hardwareMap);
            RobotLog.ii(TAG, "Follower created");

            if (Memory.autoRan) {
                RobotLog.ii(TAG, "Autonomous was run - using stored position: (%.2f, %.2f)",
                    Memory.robotAutoX, Memory.robotAutoY);
                follower.setStartingPose(new Pose(Memory.robotAutoX, Memory.robotAutoY, 0));
            } else {
                RobotLog.ii(TAG, "No autonomous run - using default position: (72, 72)");
                follower.setStartingPose(new Pose(72, 72, 0));
            }

            // Note: This overrides the previous pose - might be intentional
            follower.setStartingPose(new Pose(90, 72, 0));
            RobotLog.ii(TAG, "Final starting pose set to: (90, 72, 0°)");

            follower.startTeleOpDrive();
            RobotLog.ii(TAG, "TeleOp drive mode activated");

            follower.update();
            controller = new PIDController(p, i, d);
            RobotLog.ii(TAG, "PID controller refreshed");

            Memory.autoRan = false;
            RobotLog.ii(TAG, "Memory.autoRan reset to false");

            // Start monitoring
            teleopTimer.reset();
            RobotLog.ii(TAG, "=== Teleop ready - driver control active ===");

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error during teleop start: %s", e.getMessage());
            throw e;
        }
    }

    @Override
    public void init_loop() {
        // Alliance selection with logging
        boolean previousAlliance = Memory.allianceRed;

        if (gamepad1.a) {
            Memory.allianceRed = true;
        } else if (gamepad1.b) {
            Memory.allianceRed = false;
        }

        // Log alliance changes
        if (Memory.allianceRed != previousAlliance) {
            RobotLog.ii(TAG, "Alliance selection changed to: %s", Memory.allianceRed ? "RED" : "BLUE");
        }

        telemetry.addData("Alliance", Memory.allianceRed ? "Red" : "Blue");

        // Update shooter target based on alliance
        double previousShooterY = shooterY;
        if (Memory.allianceRed) {
            shooterY = 10;
        } else {
            shooterY = 135;
        }

        // Log shooter target changes
        if (shooterY != previousShooterY) {
            RobotLog.ii(TAG, "Shooter Y target updated: %.1f -> %.1f (Alliance: %s)",
                previousShooterY, shooterY, Memory.allianceRed ? "RED" : "BLUE");
        }
    }

    @Override
    public void loop() {
        loopTimer.reset();
        loopCount++;
        long currentTime = System.currentTimeMillis();

        try {
            // Drive control with logging
            double multiplier = 1;
            if (gamepad1.left_trigger != 0) {
                multiplier = 0.3;
                if (LoggingConfig.ENABLE_DEBUG_LOGGING && loopCount % LoggingConfig.PERIODIC_LOG_DIVIDER == 0) {
                    RobotLog.dd(TAG, "Precision mode active (multiplier: %.1f)", multiplier);
                }
            }

            double drive = -gamepad1.left_stick_y * multiplier;
            double strafe = -gamepad1.left_stick_x * multiplier;
            double turn = -gamepad1.right_stick_x * multiplier;

            follower.setTeleOpDrive(drive, strafe, turn, true);
            follower.update();

            // Position reset logging
            if (gamepad1.dpad_up) {
                RobotLog.ii(TAG, "Position reset triggered - setting pose to (72, 72, 0)");
                follower.setStartingPose(new Pose(72, 72, 0));
            }

            // Auto-turret toggle logging
            if (gamepad1.xWasPressed()) {
                stopAutoTurret = !stopAutoTurret;
                RobotLog.ii(TAG, "Auto-turret toggled: %s", stopAutoTurret ? "DISABLED" : "ENABLED");
            }

            // Latch control with logging
            boolean latchOpen = gamepad1.y;
            double currentLatchPos = latch.getPosition();
            double targetLatchPos = latchOpen ? 1 : 0;

            if (LoggingConfig.DEBUG_MODE && Math.abs(currentLatchPos - targetLatchPos) > 0.1) {
                RobotLog.dd(TAG, "Latch position change: %.1f -> %.1f (%s)",
                    currentLatchPos, targetLatchPos, latchOpen ? "OPEN" : "CLOSED");
            }
            latch.setPosition(targetLatchPos);

            // Get robot position and calculate shooting parameters
            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeading = follower.getPose().getHeading();

            double dx = shooterX - robotX;
            double dy = shooterY - robotY;
            double distance = Math.sqrt(dx*dx + dy*dy);
            double targetAngleRad = Math.atan2(dy, dx);
            double targetAngleDeg = Math.toDegrees(targetAngleRad) - Math.toDegrees(robotHeading);

            // Log robot state every 5 seconds
            if (LoggingConfig.ENABLE_TELEOP_LOGGING && currentTime - lastStatusLog > 5000) {
                RobotLog.ii(TAG, "=== Teleop Status (Loop %d) ===", loopCount);
                RobotLog.ii(TAG, "Robot position: (%.1f, %.1f, %.1f°)", robotX, robotY, Math.toDegrees(robotHeading));
                RobotLog.ii(TAG, "Target: (%.1f, %.1f), Distance: %.1f", shooterX, shooterY, distance);
                RobotLog.ii(TAG, "Runtime: %.1f seconds", teleopTimer.seconds());
                RobotLog.ii(TAG, "%s", LoggingConfig.getConfigurationSummary());
                lastStatusLog = currentTime;
            }

            // Manual turret offset control with logging
            double previousOffset = turretOffset;
            if (turretOffset <= 45 && turretOffset >= -45) {
                if (gamepad1.dpad_right && turretOffset > -45) {
                    turretOffset -= 1;
                }
                if (gamepad1.dpad_left && turretOffset < 45) {
                    turretOffset += 1;
                }
            }

            // Log turret offset changes
            if (LoggingConfig.DEBUG_MODE && turretOffset != previousOffset) {
                RobotLog.dd(TAG, "Turret offset adjusted: %.1f° -> %.1f°", previousOffset, turretOffset);
            }
            telemetry.addData("Target Angle", targetAngleDeg);

            // Apply turret offset and constraints
            double rawTargetAngle = targetAngleDeg;
            targetAngleDeg += turretOffset;
            telemetry.addData("TurretOffset", turretOffset);

            // Apply angle constraints with logging
            double constrainedAngle = Math.max(targetAngleDeg, -30);
            constrainedAngle = Math.min(constrainedAngle, 240);

            if (constrainedAngle != targetAngleDeg) {
                RobotLog.ww(TAG, "Turret angle constrained: %.1f° -> %.1f°", targetAngleDeg, constrainedAngle);
            }
            targetAngleDeg = constrainedAngle;

            double turretPos = ((double)turret.getCurrentPosition()) / TICKS_PER_DEGREES;
            telemetry.addData("Turret Pos", turretPos);

            // Turret PID control with logging
            double turretError = targetAngleDeg - turretPos;
            double turretPower = controllerTurret.calculate(turretPos, targetAngleDeg);
            telemetry.addData("TurretPower", turretPower);
            telemetry.addData("stopAutoTurret", stopAutoTurret);

            // Log turret control details periodically
            if (LoggingConfig.ENABLE_DEBUG_LOGGING && loopCount % 50 == 0) {
                RobotLog.dd(TAG, "Turret - Current: %.1f°, Target: %.1f°, Error: %.1f°, Power: %.3f, Auto: %s",
                    turretPos, targetAngleDeg, turretError, turretPower, !stopAutoTurret ? "ON" : "OFF");
            }

            if (!stopAutoTurret) {
                turret.setPower(turretPower);
            } else {
                turret.setPower(0); // Explicitly stop turret when auto is disabled
            }
            // Intake control with logging
            double intakePower = gamepad1.right_trigger;
            if (LoggingConfig.DEBUG_MODE && intakePower > 0.1) {
                RobotLog.dd(TAG, "Intake running at %.2f power", intakePower);
            }
            intake.setPower(intakePower);

            // Shooter system control with distance-based auto-aim
            if (distance > 0 && distance < 180) {
                double newTarget = RPM.get(distance);
                double newHoodAngle = angle.get(distance);

                // Log shooter parameter changes
                if (LoggingConfig.ENABLE_SHOOTER_LOGGING && Math.abs(newTarget - target) > 10) {
                    RobotLog.dd(TAG, "Shooter target updated: %.1f -> %.1f RPM (Distance: %.1f)",
                        target, newTarget, distance);
                }

                target = newTarget;
                hood.setPosition(newHoodAngle);

                // Log hood position changes
                if (LoggingConfig.ENABLE_DEBUG_LOGGING && loopCount % 100 == 0) {
                    RobotLog.dd(TAG, "Hood angle set to %.3f for distance %.1f", newHoodAngle, distance);
                }
            } else {
                if (distance >= 180 && loopCount % 100 == 0) {
                    RobotLog.ww(TAG, "Target distance out of range: %.1f (max: 180)", distance);
                }
            }

            // Shooter PID control
            controller.setPID(p, i, d);
            double presentVoltage = volt.getVoltage();

            // Log voltage warnings
            if (presentVoltage < 11.5 && loopCount % 250 == 0) {
                RobotLog.ww(TAG, "Low battery voltage: %.2fV", presentVoltage);
            }

            vel = shooterb.getVelocity() * (2 * Math.PI / 28);
            double shooterError = target - vel;
            double pid = controller.calculate(vel, target);
            pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));

            // Shooter enable/disable logic with logging
            boolean shooterEnabled = !gamepad1.a || robotX >= 40;
            if (shooterEnabled) {
                double shooterPower = (pid + f * target) / presentVoltage;
                shooterb.setPower(shooterPower);
                shootert.setPower(-shooterPower);

                // Log shooter performance periodically
                if (LoggingConfig.ENABLE_SHOOTER_LOGGING && loopCount % 100 == 0 && target > 0) {
                    RobotLog.dd(TAG, "Shooter - Target: %.1f RPM, Current: %.1f RPM, Error: %.1f, Power: %.3f",
                        target, vel, shooterError, shooterPower);
                }
            } else {
                shootert.setPower(0);
                shooterb.setPower(0);

                if (LoggingConfig.ENABLE_DEBUG_LOGGING && loopCount % 200 == 0) {
                    RobotLog.dd(TAG, "Shooter disabled - Safety interlock active (A pressed: %s, X pos: %.1f)",
                        gamepad1.a, robotX);
                }
            }

            // Reverse intake control
            if (gamepad1.dpad_down) {
                intake.setPower(-1);
                RobotLog.dd(TAG, "Reverse intake activated");
            }

//        if (gamepad1.dpad_left)
//        telemetry.addData("Turret angle: ", Math.toDegrees(turretAngle));
            telemetry.addData("Distance: ", distance);
            telemetry.addData("x: ", robotX);
            telemetry.addData("y: ", robotY);
            telemetry.addData("Heading", Math.toDegrees(robotHeading));
            telemetry.addData("RPM: ", RPM.get(distance));
            telemetry.addData("Angle: ", angle.get(distance));
            telemetry.update();

            // Performance monitoring
            long loopTime = (long) loopTimer.milliseconds();
            if (loopTime > LoggingConfig.LOOP_TIME_WARNING_MS) {
                RobotLog.ww(TAG, "Slow loop detected: %dms (Loop %d)", loopTime, loopCount);
            }

            // Periodic performance report
            if (LoggingConfig.ENABLE_PERFORMANCE_LOGGING && loopCount % 500 == 0) { // Every ~10 seconds
                double avgLoopTime = (teleopTimer.milliseconds() * 1000) / loopCount;
                RobotLog.ii(TAG, "Performance - Avg loop time: %.2fms over %d loops (%.1f seconds)",
                    avgLoopTime, loopCount, teleopTimer.seconds());
            }

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error in teleop loop %d: %s", loopCount, e.getMessage());

            // Emergency shutdown on critical errors
            try {
                shooterb.setPower(0);
                shootert.setPower(0);
                intake.setPower(0);
                turret.setPower(0);
                RobotLog.ee(TAG, "Emergency motor shutdown completed");
            } catch (Exception shutdownError) {
                RobotLog.ee(TAG, "Error during emergency shutdown: %s", shutdownError.getMessage());
            }
        }
    }
}