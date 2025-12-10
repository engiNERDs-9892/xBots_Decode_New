// TODO: NEED TO UPDATE FLYWHEEL SYNC AFTER KICKER MOVEMENT

package OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import OpModes.Main.Components.Turret;
import OpModes.Main.Components.Launcher;
import OpModes.Main.Components.Spindexer;
import OpModes.Main.Components.DriveTrain;

@TeleOp(name = "TeleOpMain", group = "Linear OpMode")
public class TeleOpMain extends LinearOpMode {
    // Components
    private Turret turret;
    private Launcher launcher;
    private Spindexer spindexer;
    private DriveTrain driveTrain;

    @Override
    public void runOpMode() {
        // Initialize components
        turret = new Turret();
        turret.initialize(hardwareMap, telemetry);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        spindexer = new Spindexer();
        spindexer.initialize(hardwareMap, telemetry, this);

        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Update launcher (flywheel)
            launcher.update();

            // Update turret alignment - exit OpMode if limelight not connected (matches original behavior)
            if (!turret.update()) {
                return;
            }
            telemetry.update();

            // Update drive train
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            driveTrain.update(forward, right, rotate, gamepad1.cross);
            telemetry.update();

            // Update spindexer
            boolean gamepadA = gamepad1.a;
            boolean gamepadB = gamepad1.b;
            boolean gamepadX = gamepad1.x;
            boolean gamepadY = gamepad1.y;
            boolean gamepadLeftBumper = gamepad1.left_bumper;

            // Handle shooting button - start flywheel when X is pressed (before shooting sequence)
            if (gamepadX && !spindexer.isPrevX()) {
                launcher.setSpinning(true);
                launcher.update(); // Update flywheel power immediately
            }

            spindexer.update(gamepadA, gamepadB, gamepadX, gamepadY, gamepadLeftBumper);

            // Stop flywheel after 3 shots
            if (spindexer.shouldStopFlywheel()) {
                launcher.setSpinning(false);
            }

            telemetry.update();
            idle();
        }
    }
}
