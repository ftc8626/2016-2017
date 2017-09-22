package org.firstinspires.ftc.teamcode.Other_Code.color_sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Stephen McConnell on 1/7/2017.
 */

@TeleOp(name = "passive", group = "color")
@Disabled
public class passive_color_two_I2C_Device extends OpMode {
   byte[] colorAcache;
   byte[] colorBcache;

    I2cDevice colorA;
    I2cDevice colorB;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorBreader;

    boolean LEDState = true;


    @Override
    public void init (){
        colorA = hardwareMap.i2cDevice.get("color_Sensor_A");
        colorB = hardwareMap.i2cDevice.get("colorB");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x3c),false);
        colorBreader = new I2cDeviceSynchImpl(colorB, I2cAddr.create8bit(0x3c),false);

        colorAreader.engage();
        colorBreader.engage();
    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){

        if (LEDState){
            colorAreader.write8(3,1);
            colorBreader.write8(3,1);
        }
        else {
            colorAreader.write8(3,0);
            colorBreader.write8(3,0);
        }
    }
    @Override
    public void loop(){
        colorAcache = colorAreader.read(0x04,1);
        colorBcache = colorBreader.read(0x04,1);

        telemetry.addData("1 #R", colorAcache[0] & 0xFF);
        telemetry.addData("2 #L", colorBcache[0] & 0xFF);

        //don't know what it does. will test when we have 2 color sensors.

//        telemetry.addData("3 A", colorAreader.getI2cAddress().get8Bit());
//        telemetry.addData("4 A", colorBreader.getI2cAddress().get8Bit());
    }

}
