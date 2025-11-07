package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Stub implementation of IMU interface for FTC SDK development
 */
public interface IMU extends HardwareDevice {
    
    /**
     * Parameters for initializing the IMU
     */
    public static class Parameters {
        public RevHubOrientationOnRobot orientationOnRobot;
        
        public Parameters(RevHubOrientationOnRobot orientationOnRobot) {
            this.orientationOnRobot = orientationOnRobot;
        }
    }
    
    /**
     * Initialize the IMU with the given parameters
     */
    void initialize(Parameters parameters);
    
    /**
     * Reset the yaw angle to zero
     */
    void resetYaw();
    
    /**
     * Get the current angular orientation as yaw, pitch, roll angles
     */
    YawPitchRollAngles getRobotYawPitchRollAngles();
    
    /**
     * Get the current angular orientation
     */
    Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit);
    
    /**
     * Get the yaw angle in the specified angle unit
     */
    double getRobotYawAngle(AngleUnit angleUnit);
}