package com.qualcomm.hardware.limelightvision;

/**
 * Stub class for FTC LLStatus
 */
public class LLStatus {
    
    public boolean isValid() {
        return true; // Stub implementation
    }
    
    public String getName() {
        return "Limelight"; // Stub implementation
    }
    
    public double getTemp() {
        return 35.0; // Stub implementation
    }
    
    public double getCpu() {
        return 15.0; // Stub implementation
    }
    
    public double getFps() {
        return 30.0; // Stub implementation
    }
    
    public int getPipelineIndex() {
        return 0; // Stub implementation
    }
    
    public String getPipelineType() {
        return "detector"; // Stub implementation
    }
    
    public String toString() {
        return "LLStatus: OK"; // Stub implementation
    }
}