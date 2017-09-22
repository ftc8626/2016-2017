/*
Modern Robotics ODS Encoder Example1
Created 9/20/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2 Beta
Reuse permitted with credit where credit is due

Configuration:
Optical Distance Sensor named "ods1"

This program can be run without a battery and Power Destitution Module.

View the video about this at https://youtu.be/EuDYJPGOOPI.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode.Other_Code.Optical_Distance_Sensor.MR_Code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "ODS Enc 1", group = "MRI")
@Disabled
public class MRI_ODS_Encoder_1 extends OpMode {

    OpticalDistanceSensor ods1;
    DeviceInterfaceModule CDI;

    //sensor value between 0 and 1023
    int raw1;
    int state = 0;
    int count = 0;

    @Override
    public void init() {
        ods1 = hardwareMap.opticalDistanceSensor.get("ods1");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
    }

    @Override
    public void loop() {

        raw1 = (int) (ods1.getLightDetected()*1023);

        if(raw1 > 400){ //Adjust this test value to be half way between the white and black value for this sensor.
            CDI.setLED(0, true);
            if(state == 1)
                count++;
            state = 0;
        }else{
            CDI.setLED(0, false);
            if(state == 0)
                count++;
            state = 1;

        }

        telemetry.addData("ODS1", raw1);
        telemetry.addData("State", state);
        telemetry.addData("Count", count);

    }

    @Override
    public void stop() {

    }

}