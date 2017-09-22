package org.firstinspires.ftc.teamcode.Other_Code.color_sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Cash America on 1/7/2017.
 */
@TeleOp(name = "Passive_one", group = "color")
@Disabled
public class Passive_color_one_I2C_Device extends OpMode {
    byte[] colorCache;

    I2cDevice color;
    I2cDeviceSynch colorReader;

    boolean LEDState = true;

    @Override
    public void init(){
        color = hardwareMap.i2cDevice.get("color");
        colorReader = new I2cDeviceSynchImpl(color, I2cAddr.create8bit(0x3c), false);
        colorReader.engage();
    }

    @Override
    public void init_loop(){}

    @Override
    public void start(){
        if (LEDState){
            colorReader.write8(3,1);
        }
        else {
            colorReader.write8(3,0);
        }
    }

    @Override
    public void loop(){
        colorCache = colorReader.read(0x04,1);
        telemetry.addData("Passive reading", colorCache[0] & 0xFF);
    }

    @Override
    public void stop(){}
}
