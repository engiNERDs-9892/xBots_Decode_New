package com.qualcomm.robotcore.hardware;

/**
 * Stub class for FTC DcMotor
 */
public interface DcMotor {
    
    enum Direction {
        FORWARD, REVERSE
    }
    
    enum RunMode {
        RUN_WITHOUT_ENCODER,
        RUN_USING_ENCODER,
        RUN_TO_POSITION,
        STOP_AND_RESET_ENCODER
    }
    
    enum ZeroPowerBehavior {
        BRAKE, FLOAT
    }
    
    void setPower(double power);
    
    double getPower();
    
    void setDirection(Direction direction);
    
    Direction getDirection();
    
    void setMode(RunMode mode);
    
    RunMode getMode();
    
    void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior);
    
    ZeroPowerBehavior getZeroPowerBehavior();
    
    int getCurrentPosition();
    
    void setTargetPosition(int position);
    
    int getTargetPosition();
    
    boolean isBusy();
}