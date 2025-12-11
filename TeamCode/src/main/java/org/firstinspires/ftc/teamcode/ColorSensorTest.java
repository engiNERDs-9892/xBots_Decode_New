package org.firstinspires.ftc.teamcode;
import android.text.Spannable;

import org.firstinspires.ftc.teamcode.color_sens_stuff;
import org.firstinspires.ftc.teamcode.util.ledstuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ColorSensorTest extends OpMode {
    color_sens_stuff colorTest = new color_sens_stuff();
    ledstuff ledTest = new ledstuff();
    color_sens_stuff.DetectedColor detectedColor;



    @Override
    public void init(){
        colorTest.init(hardwareMap);
        ledTest.init(hardwareMap);
    }
    @Override
    public void loop(){
        detectedColor = colorTest.getDetectedColor(telemetry);
        telemetry.addData("Color",detectedColor);
        if(detectedColor == color_sens_stuff.DetectedColor.GREEN){
            ledTest.setGreenLed(true);
            ledTest.setRedLed(false);
        }
        else if(detectedColor == color_sens_stuff.DetectedColor.PURP){
            ledTest.setGreenLed(false);
            ledTest.setRedLed(true);
        }
        else if(detectedColor == color_sens_stuff.DetectedColor.VOID){
            ledTest.setGreenLed(false);
            ledTest.setRedLed(false);
        }
    }
}
