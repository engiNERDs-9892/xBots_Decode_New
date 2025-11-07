package com.qualcomm.robotcore.util;

/**
 * Stub class for FTC ElapsedTime
 */
public class ElapsedTime {
    
    private long startTime;
    
    public ElapsedTime() {
        reset();
    }
    
    public void reset() {
        startTime = System.nanoTime();
    }
    
    public double seconds() {
        return (System.nanoTime() - startTime) / 1000000000.0;
    }
    
    public double milliseconds() {
        return (System.nanoTime() - startTime) / 1000000.0;
    }
    
    public long nanoseconds() {
        return System.nanoTime() - startTime;
    }
    
    @Override
    public String toString() {
        return String.format("%.1f", seconds());
    }
    
    /**
     * Get the current time in seconds (alias for seconds())
     */
    public double time() {
        return seconds();
    }
}