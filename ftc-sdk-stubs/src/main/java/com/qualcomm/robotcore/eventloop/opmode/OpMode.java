package com.qualcomm.robotcore.eventloop.opmode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Stub class for FTC OpMode
 */
public abstract class OpMode {
    
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    
    public abstract void init();
    
    public abstract void loop();
    
    public void init_loop() {
        // Stub implementation
    }
    
    public void start() {
        // Stub implementation
    }
    
    public void stop() {
        // Stub implementation
    }
}