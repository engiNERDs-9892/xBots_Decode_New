package org.firstinspires.ftc.robotcore.external.navigation;

/**
 * Stub class for FTC Pose3D
 */
public class Pose3D {
    
    public double x;
    public double y; 
    public double z;
    
    public Pose3D() {
        this(0, 0, 0);
    }
    
    public Pose3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    
    @Override
    public String toString() {
        return String.format("Pose3D(%.2f, %.2f, %.2f)", x, y, z);
    }
}