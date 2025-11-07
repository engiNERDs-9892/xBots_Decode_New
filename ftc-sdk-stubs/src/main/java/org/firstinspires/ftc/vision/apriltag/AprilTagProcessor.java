package org.firstinspires.ftc.vision.apriltag;

import java.util.List;
import java.util.ArrayList;

/**
 * Stub class for FTC AprilTagProcessor
 */
public class AprilTagProcessor {
    
    public static class Builder {
        
        public Builder setDrawAxes(boolean drawAxes) {
            return this; // Stub implementation
        }
        
        public Builder setDrawCubeProjection(boolean drawCube) {
            return this; // Stub implementation
        }
        
        public Builder setDrawTagOutline(boolean drawOutline) {
            return this; // Stub implementation
        }
        
        public Builder setTagFamily(AprilTagFamily family) {
            return this; // Stub implementation
        }
        
        public Builder setTagLibrary(AprilTagLibrary library) {
            return this; // Stub implementation
        }
        
        public Builder setOutputUnits(DistanceUnit lengthUnit, AngleUnit angleUnit) {
            return this; // Stub implementation
        }
        
        public AprilTagProcessor build() {
            return new AprilTagProcessor(); // Stub implementation
        }
    }
    
    public enum AprilTagFamily {
        TAG_16h5, TAG_25h9, TAG_36h11
    }
    
    public static class AprilTagLibrary {
        public static AprilTagLibrary getCenterStageTagLibrary() {
            return new AprilTagLibrary(); // Stub implementation
        }
        
        public static AprilTagLibrary getClassicTagLibrary() {
            return new AprilTagLibrary(); // Stub implementation
        }
    }
    
    public enum DistanceUnit {
        MM, CM, METER, INCH
    }
    
    public enum AngleUnit {
        DEGREES, RADIANS
    }
    
    public static AprilTagProcessor easyCreateWithDefaults() {
        return new AprilTagProcessor(); // Stub implementation
    }
    
    public List<AprilTagDetection> getDetections() {
        return new ArrayList<>(); // Stub implementation
    }
    
    public List<AprilTagDetection> getFreshDetections() {
        return new ArrayList<>(); // Stub implementation
    }
}