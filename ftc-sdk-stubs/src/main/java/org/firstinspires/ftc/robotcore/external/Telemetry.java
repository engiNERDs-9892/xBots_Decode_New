package org.firstinspires.ftc.robotcore.external;

/**
 * Stub class for FTC Telemetry
 */
public interface Telemetry {
    
    Item addData(String caption, Object value);
    
    Item addData(String caption, String format, Object... args);
    
    Line addLine();
    
    Line addLine(String lineCaption);
    
    void clear();
    
    void clearAll();
    
    boolean update();
    
    void speak(String text);
    
    void setMsTransmissionInterval(int msTransmissionInterval);
    
    interface Item {
        Item setValue(String format, Object... args);
        Item setValue(Object value);
        String getCaption();
        Object getValue();
    }
    
    interface Line {
        Line addData(String caption, Object value);
        Line addData(String caption, String format, Object... args);
    }
}