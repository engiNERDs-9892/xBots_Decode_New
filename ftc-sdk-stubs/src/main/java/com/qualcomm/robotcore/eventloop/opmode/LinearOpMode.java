package com.qualcomm.robotcore.eventloop.opmode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Stub class for FTC LinearOpMode
 */
public abstract class LinearOpMode extends OpMode {
    
    public abstract void runOpMode() throws InterruptedException;
    
    public final void waitForStart() {
        // Stub implementation
    }
    
    public final boolean isStarted() {
        return true; // Stub implementation
    }
    
    public final boolean isStopRequested() {
        return false; // Stub implementation
    }
    
    public final void sleep(long milliseconds) {
        // Stub implementation
    }
    
    public final boolean opModeIsActive() {
        return true; // Stub implementation
    }
    
    public final boolean opModeInInit() {
        return false; // Stub implementation - returns false when OpMode is ready to start
    }
    
    // Override the abstract methods from OpMode (not needed for LinearOpMode)
    @Override
    public final void init() {
        // LinearOpMode doesn't use init()
    }
    
    @Override
    public final void loop() {
        // LinearOpMode doesn't use loop()
    }
}