package org.firstinspires.ftc.robotcore.external.hardware.camera;

/**
 * Stub class for FTC WebcamName
 */
public class WebcamName {
    
    private String name;
    
    public WebcamName(String name) {
        this.name = name;
    }
    
    public String getName() {
        return name;
    }
    
    @Override
    public String toString() {
        return name;
    }
}