package org.firstinspires.ftc.teamcode.Other_Code.color_sensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Stephen McConnell on 1/11/2017.
 */

@TeleOp(name = "Passive_Code", group = "Color")
@Disabled
public class Passive_Color_Code_Red extends LinearOpMode {
    ColorSensor colorSensor2;
    ColorSensor colorSensor3;
    DeviceInterfaceModule CDI;

    @Override
    public void runOpMode() throws InterruptedException{
        colorSensor2 = hardwareMap.colorSensor.get("color 2");
        colorSensor3 = hardwareMap.colorSensor.get("color 3");
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x3a));
        colorSensor3.setI2cAddress(I2cAddr.create8bit(0x3e));

        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        boolean LEDState = false;

        waitForStart();
        colorSensor2.enableLed(LEDState);
        colorSensor3.enableLed(LEDState);

        float hsvValues[] = {0, 0, 0};

        while (opModeIsActive()){

            Color.RGBToHSV(colorSensor2.red()*8, colorSensor2.green()*8, colorSensor2.blue()*8, hsvValues);

            telemetry.addData("L:Red", colorSensor2.red());
            telemetry.addData("L:Blue", colorSensor2.blue());

            telemetry.addData("R:Red", colorSensor3.red());
            telemetry.addData("R:Blue", colorSensor3.blue());


            telemetry.update();

            if (colorSensor2.red()> colorSensor2.blue()){
                CDI.setLED(1,true);
                CDI.setLED(0, false);
            } else if (colorSensor2.blue()> colorSensor2.red()){
                CDI.setLED(1,false);
                CDI.setLED(0, true);
            } else if (colorSensor3.red()> colorSensor3.blue()){
                CDI.setLED(1,false);
                CDI.setLED(0, true);
            } else if (colorSensor3.blue()> colorSensor3.red()) {
                CDI.setLED(1, true);
                CDI.setLED(0, false);
            }else{
                {
                    CDI.setLED(1, false);
                    CDI.setLED(0,false);
                }
            }
        }
    }
}
