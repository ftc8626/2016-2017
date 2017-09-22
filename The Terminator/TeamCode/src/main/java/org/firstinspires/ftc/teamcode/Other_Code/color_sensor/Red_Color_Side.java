package org.firstinspires.ftc.teamcode.Other_Code.color_sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen McConnell on 1/13/2017.
 */
public class Red_Color_Side {
    public ColorSensor colorSensorR;
    public DeviceInterfaceModule CDI;

    public boolean LEDBState = false;

    HardwareMap hwMap       = null;
    private ElapsedTime period  = new ElapsedTime();


    public void init (HardwareMap ahwMap){

        hwMap = ahwMap;

        colorSensorR = ahwMap.colorSensor.get("color 3");

        colorSensorR.setI2cAddress(I2cAddr.create8bit(0x3e));

        CDI = ahwMap.deviceInterfaceModule.get("Device Interface Module 1");

        boolean LEDState = false;

    }
}
