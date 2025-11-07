package org.firstinspires.ftc.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Stub class for FTC AprilTagDetection
 */
public class AprilTagDetection {
    
    public int id;
    public double centerX;
    public double centerY;
    public Pose3D robotPose;
    public Pose3D rawPose;
    public AprilTagMetadata metadata;
    public AprilTagPose ftcPose;
    public Point center;
    
    public AprilTagDetection() {
        this.robotPose = new Pose3D();
        this.rawPose = new Pose3D();
        this.metadata = new AprilTagMetadata();
        this.ftcPose = new AprilTagPose();
        this.center = new Point();
    }
    
    public AprilTagDetection(int id, double centerX, double centerY) {
        this();
        this.id = id;
        this.centerX = centerX;
        this.centerY = centerY;
        this.center.x = centerX;
        this.center.y = centerY;
    }
    
    public static class AprilTagMetadata {
        public String name;
        public double size;
        public DistanceUnit distanceUnit;
        
        public AprilTagMetadata() {
            this.name = "Unknown";
            this.size = 6.0; // Default 6 inch tag
            this.distanceUnit = DistanceUnit.INCH;
        }
    }
    
    public static class AprilTagPose {
        public double x;
        public double y;
        public double z;
        public double pitch;
        public double roll;
        public double yaw;
        public double range;
        public double bearing;
        public double elevation;
        
        public AprilTagPose() {
            // Default stub values
        }
    }
    
    public static class Point {
        public double x;
        public double y;
        
        public Point() {
            this.x = 0;
            this.y = 0;
        }
        
        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
    
    public enum DistanceUnit {
        MM, CM, METER, INCH
    }
}