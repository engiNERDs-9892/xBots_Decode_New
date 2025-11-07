package org.firstinspires.ftc.robotcore.external.navigation;

/**
 * Stub implementation of YawPitchRollAngles for FTC SDK development
 */
public class YawPitchRollAngles {
    public final double yaw;
    public final double pitch;
    public final double roll;
    public final AngleUnit angleUnit;
    
    public YawPitchRollAngles(AngleUnit angleUnit, double yaw, double pitch, double roll) {
        this.angleUnit = angleUnit;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }
    
    /**
     * Get yaw angle
     */
    public double getYaw() {
        return yaw;
    }
    
    /**
     * Get yaw angle in specified unit
     */
    public double getYaw(AngleUnit unit) {
        return unit.fromUnit(angleUnit, yaw);
    }
    
    /**
     * Get pitch angle
     */
    public double getPitch() {
        return pitch;
    }
    
    /**
     * Get pitch angle in specified unit
     */
    public double getPitch(AngleUnit unit) {
        return unit.fromUnit(angleUnit, pitch);
    }
    
    /**
     * Get roll angle
     */
    public double getRoll() {
        return roll;
    }
    
    /**
     * Get roll angle in specified unit
     */
    public double getRoll(AngleUnit unit) {
        return unit.fromUnit(angleUnit, roll);
    }
    
    /**
     * Get angle unit
     */
    public AngleUnit getAngleUnit() {
        return angleUnit;
    }
    
    @Override
    public String toString() {
        return String.format("YPR: (%.1f, %.1f, %.1f) %s", yaw, pitch, roll, angleUnit);
    }
}