package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

import org.firstinspires.ftc.teamcode.util.LoggingConfig;

@Autonomous()
public class AutonLinear extends LinearOpMode {
    private static final String TAG = "AutonLinear";


    TurtleRobot robot = new TurtleRobot(this);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static int target = 430;
    private int pathState;
    private static double vel = 0;

    // Logging and monitoring
    private ElapsedTime autonTimer = new ElapsedTime();
    private int pathExecutionCount = 0;
    private long lastHealthCheckTime = 0;
    private final Pose Start = new Pose(28.5, 128, Math.toRadians(135));
    private final Pose ScorePosition = new Pose(60  , 85, Math.toRadians(135));
    private final Pose Grab1 = new Pose(48, 85, Math.toRadians(180));
    private final Pose Collect1 = new Pose(19, 85, Math.toRadians(180));
    private final Pose Grab2 = new Pose(48, 60, Math.toRadians(180));
    private final Pose Collect2 = new Pose(12, 60, Math.toRadians(180));
    private final Pose Grab3 = new Pose(48, 36, Math.toRadians(180));
    private final Pose Collect3 = new Pose(12, 36, Math.toRadians(180));
    private final Pose byebye = new Pose(50, 70, Math.toRadians(90));
    private final Pose byebye2 = new Pose(50, 69, Math.toRadians(90));
    private Path PreloadShoot;
    private PathChain Goto1, Pickup1, Shoot1, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, tatawireless, tatawireless2;
    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void runOpMode() {
        RobotLog.ii(TAG, "=== AutonLinear OpMode initialization started ===");
        long initStartTime = System.currentTimeMillis();

        try {
            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();
            autonTimer.reset();
            RobotLog.ii(TAG, "Timers initialized");

            robot.init(hardwareMap);
            RobotLog.ii(TAG, "TurtleRobot initialization completed");

            robot.hood.setPosition(0.3);
            RobotLog.ii(TAG, "Hood servo set to initial position: 0.3");

            follower = Constants.createFollower(hardwareMap);
            RobotLog.ii(TAG, "Pedro Pathing follower created");

            follower.setStartingPose(Start);
            RobotLog.ii(TAG, "Starting pose set: (%.1f, %.1f, %.1f°)",
                Start.getX(), Start.getY(), Math.toDegrees(Start.getHeading()));

            // Log all waypoints
            logWaypoints();

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error during initialization: %s", e.getMessage());
            throw e;
        }

        RobotLog.ii(TAG, "Building autonomous paths...");
        PreloadShoot = new Path(new BezierLine(Start, ScorePosition));
        PreloadShoot.setLinearHeadingInterpolation(Start.getHeading(), ScorePosition.getHeading());
        RobotLog.ii(TAG, "PreloadShoot path created");

        Goto1 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab1))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab1.getHeading())
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Grab1, Collect1))
                .setLinearHeadingInterpolation(Grab1.getHeading(), Collect1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1, ScorePosition))
                .setLinearHeadingInterpolation(Collect1.getHeading(), ScorePosition.getHeading())
                .build();

        Goto2 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab2))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab2.getHeading())
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(Grab2, Collect2))
                .setLinearHeadingInterpolation(Grab2.getHeading(), Collect2.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2, ScorePosition))
                .setLinearHeadingInterpolation(Collect2.getHeading(), ScorePosition.getHeading())
                .build();

        Goto3 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab3))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab3.getHeading())
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(Grab3, Collect3))
                .setLinearHeadingInterpolation(Grab3.getHeading(), Collect3.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3, ScorePosition))
                .setLinearHeadingInterpolation(Collect3.getHeading(), ScorePosition.getHeading())
                .build();

        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .build();
        tatawireless2 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .build();

        long initElapsed = System.currentTimeMillis() - initStartTime;
        RobotLog.ii(TAG, "All paths built successfully in %dms", initElapsed);
        RobotLog.ii(TAG, "=== Autonomous ready - waiting for start ===");

        waitForStart();
        try {
            RobotLog.ii(TAG, "=== AUTONOMOUS STARTED ===");
            autonTimer.reset();

            Memory.autoRan = true;
            Memory.allianceRed = false;
            RobotLog.ii(TAG, "Alliance set to BLUE, Memory.autoRan = true");

            robot.intake.setPower(1);
            RobotLog.ii(TAG, "Intake started at full power");

            updateShooterPID();
            RobotLog.ii(TAG, "Initial shooter PID update completed");

            robot.latch.setPosition(0);
            RobotLog.ii(TAG, "Latch closed (position: 0)");

            RobotLog.ii(TAG, "=== PHASE 1: Preload Shot ===");
            followPath(PreloadShoot, true);

            RobotLog.ii(TAG, "Waiting for shooter to spin up (300ms)...");
            waitShoot(300);

            RobotLog.ii(TAG, "Executing preload shot sequence");
            runShooter();

            RobotLog.ii(TAG, "=== PHASE 2: First Game Piece Cycle ===");
            RobotLog.ii(TAG, "Moving to first pickup position");
            followPath(Goto1, true);

            robot.intake.setPower(1);
            RobotLog.ii(TAG, "Intake restarted, moving to collect first piece");
            followPath(Pickup1, true);

            telemetry.addData("Before crash", 1);
            telemetry.update();
            RobotLog.ii(TAG, "First piece collected, moving to shoot position");

            //waitMillis(1000);
            followPath(Shoot1, true);

//            waitShoot(300);
            RobotLog.ii(TAG, "Executing first piece shot sequence");
            runShooter();

            RobotLog.ii(TAG, "=== PHASE 3: Second Game Piece Cycle ===");
            RobotLog.ii(TAG, "Moving to second pickup position");
            followPath(Goto2, true);

            robot.intake.setPower(1);
            RobotLog.ii(TAG, "Intake started, collecting second piece");
            followPath(Pickup2, true);
            //waitMillis(200);
            robot.intake.setPower(0);
            RobotLog.ii(TAG, "Second piece collected, intake stopped");

            RobotLog.ii(TAG, "Moving to second shot position");
            followPath(Shoot2, true);

            RobotLog.ii(TAG, "Waiting for shooter to spin up (300ms)...");
            waitShoot(300);

            RobotLog.ii(TAG, "Executing second piece shot sequence");
            runShooter();

            RobotLog.ii(TAG, "=== PHASE 4: Third Game Piece Cycle ===");
            RobotLog.ii(TAG, "Moving to third pickup position");
            followPath(Goto3, true);

            robot.intake.setPower(1);
            RobotLog.ii(TAG, "Intake started, collecting third piece");
            followPath(Pickup3, true);
            //waitMillis(1000);
            robot.intake.setPower(0);
            RobotLog.ii(TAG, "Third piece collected, intake stopped");

            RobotLog.ii(TAG, "Moving to third shot position");
            followPath(Shoot3, true);

            RobotLog.ii(TAG, "Waiting for shooter to spin up (300ms)...");
            waitShoot(300);

            RobotLog.ii(TAG, "Executing third piece shot sequence");
            runShooter();


            RobotLog.ii(TAG, "=== PHASE 5: Parking and Shutdown ===");
            robot.shooterb.setPower(0);
            robot.shootert.setPower(0);
            RobotLog.ii(TAG, "Shooters powered down");

            RobotLog.ii(TAG, "Moving to parking position");
            followPath(tatawireless, true);

            Memory.robotAutoX = follower.getPose().getX();
            Memory.robotAutoY = follower.getPose().getY();
            Memory.robotHeading = follower.getPose().getHeading();
            RobotLog.ii(TAG, "Final position stored in Memory: (%.1f, %.1f, %.1f°)",
                Memory.robotAutoX, Memory.robotAutoY, Math.toDegrees(Memory.robotHeading));

            robot.shooterb.setPower(0);
            robot.shootert.setPower(0);
            robot.turret.setPower(0);
            RobotLog.ii(TAG, "All motors powered down for parking");

            double totalTime = autonTimer.seconds();
            RobotLog.ii(TAG, "=== AUTONOMOUS COMPLETED in %.2f seconds ===", totalTime);
            RobotLog.ii(TAG, "Beginning turret reset and hold position...");

            while (opModeIsActive()) {
                double turretPos = ((double)robot.turret.getCurrentPosition()) / TurtleRobot.TICKS_PER_DEGREES;
                telemetry.addData("Turret Pos", turretPos);

                double turretPower = robot.controllerTurret.calculate(turretPos, 0);
                telemetry.addData("Power", turretPower);
                robot.turret.setPower(turretPower);
                follower.update();

                // Periodic status logging during hold
                if (System.currentTimeMillis() - lastHealthCheckTime > 5000) {
                    RobotLog.ii(TAG, "Holding position - Turret: %.1f°, Target: 0°", turretPos);
                    lastHealthCheckTime = System.currentTimeMillis();
                }
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, "=== CRITICAL AUTONOMOUS ERROR ===");
            RobotLog.ee(TAG, "Exception occurred: %s", e.getMessage());
            RobotLog.ee(TAG, "Stack trace: %s", android.util.Log.getStackTraceString(e));
            RobotLog.ee(TAG, "Autonomous runtime before error: %.2f seconds", autonTimer.seconds());

            // Emergency shutdown
            robot.logEmergencyStop("Autonomous exception: " + e.getMessage());
            try {
                robot.shooterb.setPower(0);
                robot.shootert.setPower(0);
                robot.intake.setPower(0);
                robot.turret.setPower(0);
                RobotLog.ee(TAG, "Emergency motor shutdown completed");
            } catch (Exception shutdownError) {
                RobotLog.ee(TAG, "Error during emergency shutdown: %s", shutdownError.getMessage());
            }

            telemetry.addData("Exception", e.toString());
            telemetry.update();
            sleep(10000000);
        }
    }

    private void runShooter() {
        RobotLog.ii(TAG, "Shooter sequence started - opening latch and starting intake");
        robot.latch.setPosition(1);
        robot.intake.setPower(1);

        RobotLog.ii(TAG, "Waiting for shot completion (2200ms)...");
        waitShoot(2200);

        robot.latch.setPosition(0);
        robot.intake.setPower(0);
        RobotLog.ii(TAG, "Shooter sequence completed - latch closed, intake stopped");
    }

    public void followPath(PathChain path, boolean holdEnd) {
        pathExecutionCount++;
        RobotLog.ii(TAG, "Starting PathChain execution #%d (holdEnd: %s)", pathExecutionCount, holdEnd);
        long pathStartTime = System.currentTimeMillis();

        follower.followPath(path, holdEnd);
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        Memory.robotHeading = follower.getPose().getHeading();

        int updateCount = 0;
        while (follower.isBusy()) {
            follower.update();
            updateShooterPID();
            updateTurretPID();
            updateCount++;

            // Log progress every 50 updates (~1 second)
            if (LoggingConfig.ENABLE_DEBUG_LOGGING && updateCount % 50 == 0) {
                RobotLog.dd(TAG, "Path following - Updates: %d, Position: (%.1f, %.1f)",
                    updateCount, follower.getPose().getX(), follower.getPose().getY());
            }
        }

        long pathElapsed = System.currentTimeMillis() - pathStartTime;
        RobotLog.ii(TAG, "PathChain #%d completed in %dms (%d updates). Final pose: (%.1f, %.1f, %.1f°)",
            pathExecutionCount, pathElapsed, updateCount,
            Memory.robotAutoX, Memory.robotAutoY, Math.toDegrees(Memory.robotHeading));
    }

    public void followPath(Path path, boolean holdEnd) {
        pathExecutionCount++;
        RobotLog.ii(TAG, "Starting Path execution #%d (holdEnd: %s)", pathExecutionCount, holdEnd);
        long pathStartTime = System.currentTimeMillis();

        follower.followPath(path, holdEnd);
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        Memory.robotHeading = follower.getPose().getHeading();

        int updateCount = 0;
        while (follower.isBusy()) {
            follower.update();
            updateShooterPID();
            updateTurretPID();
            updateCount++;
        }

        long pathElapsed = System.currentTimeMillis() - pathStartTime;
        RobotLog.ii(TAG, "Path #%d completed in %dms (%d updates). Final pose: (%.1f, %.1f, %.1f°)",
            pathExecutionCount, pathElapsed, updateCount,
            Memory.robotAutoX, Memory.robotAutoY, Math.toDegrees(Memory.robotHeading));
    }

    public void updateShooterPID() {
        try {
            robot.controller.setPID(robot.p, robot.i, robot.d);
            double presentVoltage = robot.volt.getVoltage();
            vel = vel * robot.alpha + robot.shooterb.getVelocity() * (2 * Math.PI / 28) * (1 - robot.alpha);

            double error = target - vel;
            double pid = robot.controller.calculate(vel, target);
            pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));

            robot.shooterb.setPower(pid * 0.9);
            robot.shootert.setPower(-0.9 * pid);

            // Log detailed PID info periodically
            if (LoggingConfig.ENABLE_DEBUG_LOGGING && System.currentTimeMillis() % 1000 < 20) { // ~once per second
                RobotLog.dd(TAG, "Shooter PID - Target: %d RPM, Current: %.1f RPM, Error: %.1f, PID: %.3f, Voltage: %.2fV",
                    target, vel, error, pid, presentVoltage);
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, "Error in updateShooterPID: %s", e.getMessage());
        }
    }

    public void updateTurretPID() {
        try {
            double turretPos = ((double)robot.turret.getCurrentPosition()) / TurtleRobot.TICKS_PER_DEGREES;
            double targetAngle = 5.0;
            double error = targetAngle - turretPos;

            telemetry.addData("Turret Pos", turretPos);

            double turretPower = robot.controllerTurret.calculate(turretPos, targetAngle);
            telemetry.addData("Power", turretPower);
            robot.turret.setPower(turretPower);

            // Log turret control details periodically
            if (LoggingConfig.ENABLE_DEBUG_LOGGING && System.currentTimeMillis() % 1000 < 20) { // ~once per second
                RobotLog.dd(TAG, "Turret PID - Current: %.1f°, Target: %.1f°, Error: %.1f°, Power: %.3f",
                    turretPos, targetAngle, error, turretPower);
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, "Error in updateTurretPID: %s", e.getMessage());
        }
    }

    public void waitShoot(long sleepTimeMillis) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start + sleepTimeMillis) {
            updateShooterPID();
            updateTurretPID();
//            follower.update();
            sleep(1);
        }
    }

    public void waitMillis(long sleepTimeMillis) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start + sleepTimeMillis) {
//            follower.update();
            sleep(1);
        }
    }

    /**
     * Log all waypoint positions for debugging
     */
    private void logWaypoints() {
        RobotLog.ii(TAG, "=== Autonomous Waypoints ===");
        RobotLog.ii(TAG, "Start: (%.1f, %.1f, %.1f°)", Start.getX(), Start.getY(), Math.toDegrees(Start.getHeading()));
        RobotLog.ii(TAG, "ScorePosition: (%.1f, %.1f, %.1f°)", ScorePosition.getX(), ScorePosition.getY(), Math.toDegrees(ScorePosition.getHeading()));
        RobotLog.ii(TAG, "Grab1: (%.1f, %.1f, %.1f°)", Grab1.getX(), Grab1.getY(), Math.toDegrees(Grab1.getHeading()));
        RobotLog.ii(TAG, "Collect1: (%.1f, %.1f, %.1f°)", Collect1.getX(), Collect1.getY(), Math.toDegrees(Collect1.getHeading()));
        RobotLog.ii(TAG, "Grab2: (%.1f, %.1f, %.1f°)", Grab2.getX(), Grab2.getY(), Math.toDegrees(Grab2.getHeading()));
        RobotLog.ii(TAG, "Collect2: (%.1f, %.1f, %.1f°)", Collect2.getX(), Collect2.getY(), Math.toDegrees(Collect2.getHeading()));
        RobotLog.ii(TAG, "Grab3: (%.1f, %.1f, %.1f°)", Grab3.getX(), Grab3.getY(), Math.toDegrees(Grab3.getHeading()));
        RobotLog.ii(TAG, "Collect3: (%.1f, %.1f, %.1f°)", Collect3.getX(), Collect3.getY(), Math.toDegrees(Collect3.getHeading()));
        RobotLog.ii(TAG, "Park: (%.1f, %.1f, %.1f°)", byebye.getX(), byebye.getY(), Math.toDegrees(byebye.getHeading()));
        RobotLog.ii(TAG, "=== End Waypoints ===");
    }
}

