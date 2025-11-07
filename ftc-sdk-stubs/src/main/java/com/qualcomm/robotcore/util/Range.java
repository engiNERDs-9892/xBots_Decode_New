package com.qualcomm.robotcore.util;

/**
 * Stub class for FTC Range utility
 */
public class Range {
    
    public static double clip(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
    
    public static float clip(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
    
    public static int clip(int value, int min, int max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
    
    public static double scale(double value, double oldMin, double oldMax, double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }
}