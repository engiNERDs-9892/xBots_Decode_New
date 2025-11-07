package org.firstinspires.ftc.robotcore.external.navigation;

/**
 * Stub implementation of AngleUnit enum for FTC SDK development
 */
public enum AngleUnit {
    DEGREES,
    RADIANS;
    
    /**
     * Convert from one unit to another
     */
    public double fromUnit(AngleUnit fromUnit, double value) {
        if (fromUnit == this) {
            return value;
        }
        
        if (fromUnit == DEGREES && this == RADIANS) {
            return Math.toRadians(value);
        }
        
        if (fromUnit == RADIANS && this == DEGREES) {
            return Math.toDegrees(value);
        }
        
        return value;
    }
    
    /**
     * Convert to another unit
     */
    public double toUnit(AngleUnit toUnit, double value) {
        return toUnit.fromUnit(this, value);
    }
    
    /**
     * Normalize angle to [-180, 180] for degrees or [-π, π] for radians
     */
    public double normalize(double angle) {
        if (this == DEGREES) {
            while (angle > 180) angle -= 360;
            while (angle <= -180) angle += 360;
        } else {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle <= -Math.PI) angle += 2 * Math.PI;
        }
        return angle;
    }
}