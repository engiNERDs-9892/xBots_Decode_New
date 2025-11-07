package org.firstinspires.ftc.robotcore.external.navigation;

/**
 * Stub implementation of Orientation class for FTC SDK development
 */
public class Orientation {
    public final AxesReference axesReference;
    public final AxesOrder axesOrder;
    public final AngleUnit angleUnit;
    public final float firstAngle;
    public final float secondAngle;
    public final float thirdAngle;
    
    public Orientation(AxesReference axesReference, AxesOrder axesOrder, AngleUnit angleUnit,
                      float firstAngle, float secondAngle, float thirdAngle) {
        this.axesReference = axesReference;
        this.axesOrder = axesOrder;
        this.angleUnit = angleUnit;
        this.firstAngle = firstAngle;
        this.secondAngle = secondAngle;
        this.thirdAngle = thirdAngle;
    }
    
    /**
     * Get the first angle
     */
    public float getFirstAngle() {
        return firstAngle;
    }
    
    /**
     * Get the second angle
     */
    public float getSecondAngle() {
        return secondAngle;
    }
    
    /**
     * Get the third angle
     */
    public float getThirdAngle() {
        return thirdAngle;
    }
    
    /**
     * Convert to string representation
     */
    @Override
    public String toString() {
        return String.format("Orientation: (%.1f, %.1f, %.1f) %s %s %s", 
                           firstAngle, secondAngle, thirdAngle, 
                           axesReference, axesOrder, angleUnit);
    }
}