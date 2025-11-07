package com.qualcomm.robotcore.hardware;

/**
 * Stub class for FTC NormalizedRGBA
 */
public class NormalizedRGBA {
    
    public float red;
    public float green;
    public float blue;
    public float alpha;
    
    public NormalizedRGBA() {
        this(0, 0, 0, 0);
    }
    
    public NormalizedRGBA(float red, float green, float blue, float alpha) {
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.alpha = alpha;
    }
    
    public int toColor() {
        return android.graphics.Color.argb(
            (int)(alpha * 255),
            (int)(red * 255),
            (int)(green * 255),
            (int)(blue * 255)
        );
    }
}