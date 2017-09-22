package org.firstinspires.ftc.teamcode.Other_Code.color_sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Cash America on 1/8/2017.
 */

@TeleOp(name = "LED-test", group = "color")
@Disabled
public class Color_Sensor_LED_Test extends LinearOpMode {
    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("color 1");
        colorSensor.setI2cAddress(I2cAddr.create8bit(0x3e));

        boolean LEDState = true;

        waitForStart();

        colorSensor.enableLed(LEDState);
    }
}
