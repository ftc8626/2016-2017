package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Cash America on 1/12/2017.
 */

@TeleOp(name = "stuff", group = "stuff")
@Disabled
public class stuff extends OpMode {
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;
    ColorSensor colorSensor3;

    @Override
    public void init() {
        colorSensor1 = hardwareMap.colorSensor.get("ca");
        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x3a));
        colorSensor2 = hardwareMap.colorSensor.get("cc");
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensor3 = hardwareMap.colorSensor.get("ce");
        colorSensor3.setI2cAddress(I2cAddr.create8bit(0x3e));

        colorSensor1.getI2cAddress();
        colorSensor2.getI2cAddress();
        colorSensor3.getI2cAddress();
    }
    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
        telemetry.addData("color 1:A", colorSensor1.getI2cAddress().get8Bit());
        telemetry.addData("color 2:C", colorSensor2.getI2cAddress().get8Bit());
        telemetry.addData("color 3:E", colorSensor3.getI2cAddress().get8Bit());
        telemetry.update();
    }
    @Override
    public void stop(){}
}
