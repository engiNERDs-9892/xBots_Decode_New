package org.firstinspires.ftc.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Stub class for FTC VisionPortal
 */
public class VisionPortal {
    
    public static class Builder {
        
        public Builder setCamera(WebcamName webcam) {
            return this; // Stub implementation
        }
        
        public Builder setCamera(BuiltinCameraDirection direction) {
            return this; // Stub implementation
        }
        
        public Builder addProcessor(AprilTagProcessor processor) {
            return this; // Stub implementation
        }
        
        public Builder setCameraResolution(CameraSize size) {
            return this; // Stub implementation
        }
        
        public Builder setStreamFormat(VisionPortal.StreamFormat format) {
            return this; // Stub implementation
        }
        
        public VisionPortal build() {
            return new VisionPortal(); // Stub implementation
        }
    }
    
    public enum StreamFormat {
        YUY2, MJPEG
    }
    
    public static class CameraSize {
        public int width;
        public int height;
        
        public CameraSize(int width, int height) {
            this.width = width;
            this.height = height;
        }
    }
    
    public static VisionPortal easyCreateWithDefaults(WebcamName webcam, AprilTagProcessor processor) {
        return new VisionPortal(); // Stub implementation
    }
    
    public static VisionPortal easyCreateWithDefaults(BuiltinCameraDirection direction, AprilTagProcessor processor) {
        return new VisionPortal(); // Stub implementation
    }
    
    public void stopStreaming() {
        // Stub implementation
    }
    
    public void resumeStreaming() {
        // Stub implementation
    }
    
    public void close() {
        // Stub implementation
    }
}