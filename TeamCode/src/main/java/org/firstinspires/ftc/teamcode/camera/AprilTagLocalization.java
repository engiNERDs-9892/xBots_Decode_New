package org.firstinspires.ftc.teamcode.camera;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag based localization.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will be used to compute the robot's location and orientation.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the robot, relative to the field origin.
 * This information is provided in the "robotPose" member of the returned "detection".
 *
 * To learn about the Field Coordinate System that is defined for FTC (and used by this OpMode), see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: AprilTag Localization", group = "Concept")
@Disabled
public class AprilTagLocalization extends LinearOpMode {
    private static final String TAG = "AprilTagLocalization";

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // Logging and monitoring
    private ElapsedTime visionTimer = new ElapsedTime();
    private int detectionCount = 0;
    private int totalFramesProcessed = 0;
    private long lastPerformanceLog = 0;
    private double lastRobotX = 0, lastRobotY = 0, lastRobotYaw = 0;

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        RobotLog.ii(TAG, "=== AprilTag Localization OpMode started ===");
        long initStartTime = System.currentTimeMillis();

        try {
            initAprilTag();
            long initElapsed = System.currentTimeMillis() - initStartTime;
            RobotLog.ii(TAG, "AprilTag initialization completed in %dms", initElapsed);
        } catch (Exception e) {
            RobotLog.ee(TAG, "Error during AprilTag initialization: %s", e.getMessage());
            throw e;
        }

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        RobotLog.ii(TAG, "Waiting for start...");
        waitForStart();

        RobotLog.ii(TAG, "OpMode started - beginning AprilTag detection loop");
        visionTimer.reset();

        while (opModeIsActive()) {
            totalFramesProcessed++;

            try {
                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    RobotLog.ii(TAG, "Stopping camera streaming (DPAD_DOWN pressed)");
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    RobotLog.ii(TAG, "Resuming camera streaming (DPAD_UP pressed)");
                    visionPortal.resumeStreaming();
                }

                // Log performance metrics every 5 seconds
                long currentTime = System.currentTimeMillis();
                if (currentTime - lastPerformanceLog > 5000) {
                    logPerformanceMetrics();
                    lastPerformanceLog = currentTime;
                }

                // Share the CPU.
                sleep(20);

            } catch (Exception e) {
                RobotLog.ee(TAG, "Error in AprilTag detection loop: %s", e.getMessage());
            }
        }

        // Save more CPU resources when camera is no longer needed.
        RobotLog.ii(TAG, "Closing vision portal and shutting down");
        visionPortal.close();

        double totalRuntime = visionTimer.seconds();
        RobotLog.ii(TAG, "=== AprilTag session completed ===");
        RobotLog.ii(TAG, "Total runtime: %.2f seconds", totalRuntime);
        RobotLog.ii(TAG, "Frames processed: %d", totalFramesProcessed);
        RobotLog.ii(TAG, "Tags detected: %d", detectionCount);
        if (totalFramesProcessed > 0) {
            RobotLog.ii(TAG, "Average FPS: %.1f", totalFramesProcessed / totalRuntime);
        }
    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        RobotLog.ii(TAG, "Initializing AprilTag processor...");

        try {
            // Log camera configuration
            RobotLog.ii(TAG, "Camera configuration:");
            RobotLog.ii(TAG, "  Using: %s", USE_WEBCAM ? "Webcam" : "Phone camera");
            RobotLog.ii(TAG, "  Position: (%.1f, %.1f, %.1f) inches",
                cameraPosition.x, cameraPosition.y, cameraPosition.z);
            RobotLog.ii(TAG, "  Orientation: Yaw=%.1f°, Pitch=%.1f°, Roll=%.1f°",
                cameraOrientation.getYaw(AngleUnit.DEGREES),
                cameraOrientation.getPitch(AngleUnit.DEGREES),
                cameraOrientation.getRoll(AngleUnit.DEGREES));

            // Create the AprilTag processor.
            RobotLog.ii(TAG, "Creating AprilTag processor...");
            aprilTag = new AprilTagProcessor.Builder()

                    // The following default settings are available to un-comment and edit as needed.
                    //.setDrawAxes(false)
                    //.setDrawCubeProjection(false)
                    //.setDrawTagOutline(true)
                    //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setCameraPose(cameraPosition, cameraOrientation)

                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    // ... these parameters are fx, fy, cx, cy.

                    .build();

            RobotLog.ii(TAG, "AprilTag processor created successfully");

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

            // Create the vision portal by using a builder.
            RobotLog.ii(TAG, "Creating vision portal...");
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera).
            if (USE_WEBCAM) {
                RobotLog.ii(TAG, "Configuring for webcam: 'Webcam 1'");
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                RobotLog.ii(TAG, "Configuring for built-in camera: BACK");
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            // Choose a camera resolution. Not all cameras support all resolutions.
            //builder.setCameraResolution(new Size(640, 480));
            RobotLog.ii(TAG, "Using default camera resolution");

            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            builder.enableLiveView(true);
            RobotLog.ii(TAG, "LiveView enabled for camera monitoring");

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            //builder.setAutoStopLiveView(false);

            // Set and enable the processor.
            builder.addProcessor(aprilTag);
            RobotLog.ii(TAG, "AprilTag processor added to vision portal");

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();
            RobotLog.ii(TAG, "Vision portal built successfully");

            // Disable or re-enable the aprilTag processor at any time.
            //visionPortal.setProcessorEnabled(aprilTag, true);

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error during AprilTag initialization: %s", e.getMessage());
            RobotLog.ee(TAG, "Stack trace: %s", android.util.Log.getStackTraceString(e));
            throw e;
        }
    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        try {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int detectionsThisFrame = currentDetections.size();
            telemetry.addData("# AprilTags Detected", detectionsThisFrame);

            // Log detection events
            if (detectionsThisFrame > 0) {
                detectionCount += detectionsThisFrame;
                RobotLog.dd(TAG, "Frame %d: %d AprilTag(s) detected", totalFramesProcessed, detectionsThisFrame);
            }

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    // Extract pose information
                    double robotX = detection.robotPose.getPosition().x;
                    double robotY = detection.robotPose.getPosition().y;
                    double robotZ = detection.robotPose.getPosition().z;
                    double pitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                    double roll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                    double yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", robotX, robotY, robotZ));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", pitch, roll, yaw));

                    // Log detailed AprilTag information
                    RobotLog.ii(TAG, "AprilTag ID %d (%s) detected:", detection.id, detection.metadata.name);
                    RobotLog.ii(TAG, "  Robot pose: (%.2f, %.2f, %.2f) inches", robotX, robotY, robotZ);
                    RobotLog.ii(TAG, "  Robot orientation: P=%.1f°, R=%.1f°, Y=%.1f°", pitch, roll, yaw);
                    RobotLog.ii(TAG, "  Tag center: (%.1f, %.1f) pixels", detection.center.x, detection.center.y);

                    // Check for significant robot movement
                    double deltaX = Math.abs(robotX - lastRobotX);
                    double deltaY = Math.abs(robotY - lastRobotY);
                    double deltaYaw = Math.abs(yaw - lastRobotYaw);

                    if (deltaX > 6.0 || deltaY > 6.0 || deltaYaw > 15.0) {
                        RobotLog.ii(TAG, "Significant robot movement detected:");
                        RobotLog.ii(TAG, "  Position change: (%.1f, %.1f) -> (%.1f, %.1f)", lastRobotX, lastRobotY, robotX, robotY);
                        RobotLog.ii(TAG, "  Yaw change: %.1f° -> %.1f°", lastRobotYaw, yaw);
                    }

                    // Update last known position
                    lastRobotX = robotX;
                    lastRobotY = robotY;
                    lastRobotYaw = yaw;

                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));

                    RobotLog.ww(TAG, "Unknown AprilTag ID %d detected at pixel (%.0f, %.0f)",
                        detection.id, detection.center.x, detection.center.y);
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

        } catch (Exception e) {
            RobotLog.ee(TAG, "Error in telemetryAprilTag: %s", e.getMessage());
            telemetry.addData("ERROR", "AprilTag processing failed");
        }
    }

    /**
     * Log performance metrics for AprilTag detection
     */
    private void logPerformanceMetrics() {
        double runtime = visionTimer.seconds();
        double avgFPS = runtime > 0 ? totalFramesProcessed / runtime : 0;
        double detectionRate = totalFramesProcessed > 0 ? (double)detectionCount / totalFramesProcessed : 0;

        RobotLog.ii(TAG, "=== AprilTag Performance Metrics ===");
        RobotLog.ii(TAG, "Runtime: %.2f seconds", runtime);
        RobotLog.ii(TAG, "Frames processed: %d", totalFramesProcessed);
        RobotLog.ii(TAG, "Average FPS: %.1f", avgFPS);
        RobotLog.ii(TAG, "Total detections: %d", detectionCount);
        RobotLog.ii(TAG, "Detection rate: %.1f%% (%.2f detections/frame)",
            detectionRate * 100, detectionRate);
        RobotLog.ii(TAG, "Current robot position: (%.2f, %.2f, %.1f°)",
            lastRobotX, lastRobotY, lastRobotYaw);

        // Performance warnings
        if (avgFPS < 15) {
            RobotLog.ww(TAG, "Low FPS detected: %.1f (target >15)", avgFPS);
        }
        if (detectionRate < 0.1 && totalFramesProcessed > 100) {
            RobotLog.ww(TAG, "Low detection rate: %.1f%% - check tag visibility", detectionRate * 100);
        }
    }   // end method telemetryAprilTag()

}   // end class

