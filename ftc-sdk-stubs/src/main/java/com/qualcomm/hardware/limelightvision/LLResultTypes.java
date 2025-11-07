package com.qualcomm.hardware.limelightvision;

/**
 * Stub class for FTC LLResultTypes
 */
public class LLResultTypes {
    
    public static class BarcodeResult {
        public String data;
        public String format;
        
        public BarcodeResult() {}
        
        public String getData() { return data; }
        public String getFormat() { return format; }
    }
    
    public static class ClassifierResult {
        public String className;
        public double confidence;
        
        public ClassifierResult() {}
        
        public String getClassName() { return className; }
        public double getConfidence() { return confidence; }
    }
    
    public static class DetectorResult {
        public String className;
        public double confidence;
        public double x, y, width, height;
        
        public DetectorResult() {}
        
        public String getClassName() { return className; }
        public double getConfidence() { return confidence; }
        public double getTargetArea() { return width * height; }
    }
    
    public static class FiducialResult {
        public int fiducialId;
        public double x, y;
        public String family;
        
        public FiducialResult() {}
        
        public int getFiducialId() { return fiducialId; }
        public String getFamily() { return family; }
        public double getTargetXDegrees() { return x; }
        public double getTargetYDegrees() { return y; }
    }
    
    public static class ColorResult {
        public double x, y;
        public String color;
        
        public ColorResult() {}
        
        public double getTargetXDegrees() { return x; }
        public double getTargetYDegrees() { return y; }
        public String getColor() { return color; }
    }
}