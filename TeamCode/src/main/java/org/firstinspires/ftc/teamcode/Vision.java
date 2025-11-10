package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vision {
    private Limelight3A limelight;
    private LLStatus status;
    private LLResult result;
    private boolean detecting;
    private double targetOffsetAngle_Verticle;
    private final double limelightMountAngleDegrees = 11;
    private final double limelightLensHeightInches = 10.0;
    private final double goalHeightInches = 29.5;
    private double angleToGoalDegrees;
    private double angleToGoalRadians;
    private double distanceFromLimelightToGoalInches;
    public Vision(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        status = limelight.getStatus();
        result = limelight.getLatestResult();
        detecting = result.isValid();;
    }

    public Limelight3A getLimelight()
    {
        return this.limelight;
    }
    public LLStatus getStatus()
    {
        return this.status;
    }
    public LLResult getResult()
    {
        return this.result;
    }
    public boolean detecting()
    {
        return this.detecting;
    }
    public double findDistance()
    {
        if(this.detecting)
        {
            targetOffsetAngle_Verticle = (result.getTy());
            angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Verticle;
            angleToGoalRadians = angleToGoalDegrees * (Math.PI/180.0);
            distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            return this.distanceFromLimelightToGoalInches;
        }
        else
        {
            return 0.0;
        }
    }
    public double getTx()
    {
        return this.result.getTx();
    }
    public double getTargetVelocity()
    {
        return .2214*Math.pow(this.distanceFromLimelightToGoalInches,2) - 12.7553*this.distanceFromLimelightToGoalInches + 1216.8651;
    }



}
