package com.qualcomm.robotcore.hardware;

/**
 * Stub implementation of HardwareDevice interface for FTC SDK development
 */
public interface HardwareDevice {
    /**
     * Get the manufacturer of this device
     */
    Manufacturer getManufacturer();
    
    /**
     * Get the device name
     */
    String getDeviceName();
    
    /**
     * Get the connection info
     */
    String getConnectionInfo();
    
    /**
     * Get the version
     */
    int getVersion();
    
    /**
     * Reset the device encoders
     */
    void resetDeviceConfigurationForOpMode();
    
    /**
     * Close the device
     */
    void close();
    
    /**
     * Enum for device manufacturers
     */
    enum Manufacturer {
        UNKNOWN,
        LEGO,
        HIT_TECHNIC,
        ADAFRUIT,
        MATRIX,
        MODERN_ROBOTICS,
        LYNX,
        REV,
        OTHER
    }
}