package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Stub implementation of RevHubOrientationOnRobot for FTC SDK development
 */
public class RevHubOrientationOnRobot {
    
    /**
     * Logo facing direction
     */
    public enum LogoFacingDirection {
        UP,
        DOWN,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }
    
    /**
     * USB facing direction
     */
    public enum UsbFacingDirection {
        UP,
        DOWN,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }
    
    public final LogoFacingDirection logoFacingDirection;
    public final UsbFacingDirection usbFacingDirection;
    
    /**
     * Constructor using logo and USB facing directions
     */
    public RevHubOrientationOnRobot(LogoFacingDirection logoFacingDirection, 
                                   UsbFacingDirection usbFacingDirection) {
        this.logoFacingDirection = logoFacingDirection;
        this.usbFacingDirection = usbFacingDirection;
    }
    
    /**
     * Constructor using yaw, pitch, roll angles
     */
    public RevHubOrientationOnRobot(AngleUnit angleUnit, double yaw, double pitch, double roll) {
        // For stub purposes, default to common orientations
        this.logoFacingDirection = LogoFacingDirection.UP;
        this.usbFacingDirection = UsbFacingDirection.FORWARD;
    }
    
    /**
     * Get logo facing direction
     */
    public LogoFacingDirection getLogoFacingDirection() {
        return logoFacingDirection;
    }
    
    /**
     * Get USB facing direction
     */
    public UsbFacingDirection getUsbFacingDirection() {
        return usbFacingDirection;
    }
    
    @Override
    public String toString() {
        return String.format("RevHubOrientation: Logo=%s, USB=%s", 
                           logoFacingDirection, usbFacingDirection);
    }
}