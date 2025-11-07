package com.qualcomm.robotcore.hardware;

import android.content.Context;

/**
 * Stub class for FTC HardwareMap
 */
public class HardwareMap {
    
    public Context appContext;
    
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        return null; // Stub implementation
    }
    
    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        return null; // Stub implementation
    }
}