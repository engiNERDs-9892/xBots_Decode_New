package com.qualcomm.robotcore.hardware;

/**
 * Stub interface for FTC ColorSensor
 */
public interface ColorSensor {
    
    int red();
    
    int green();
    
    int blue();
    
    int alpha();
    
    void enableLed(boolean enable);
}