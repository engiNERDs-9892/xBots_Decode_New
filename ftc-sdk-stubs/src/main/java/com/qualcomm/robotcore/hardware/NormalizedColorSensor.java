package com.qualcomm.robotcore.hardware;

/**
 * Stub interface for FTC NormalizedColorSensor
 */
public interface NormalizedColorSensor {
    
    NormalizedRGBA getNormalizedColors();
    
    void setGain(float gain);
    
    float getGain();
}