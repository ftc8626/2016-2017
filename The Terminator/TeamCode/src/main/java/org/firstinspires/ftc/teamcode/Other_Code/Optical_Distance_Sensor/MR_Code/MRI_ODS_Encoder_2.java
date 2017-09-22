/*
Modern Robotics ODS Encoder Example2
Created 9/20/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2 Beta
Reuse permitted with credit where credit is due

Configuration:
Optical Distance Sensor named "ods1"
Optical Distance Sensor named "ods2"

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

@TeleOp(name = "ODS Enc 2", group = "MRI")
@Disabled
public class MRI_ODS_Encoder_2 extends OpMode {

    OpticalDistanceSensor ods1;
    OpticalDistanceSensor ods2;
    DeviceInterfaceModule CDI;

    //raw data from each sensor. 0-1023
    int raw1;
    int raw2;

    int state = 0;
    int preState = 0;

    //movement is a combination of the previous state and the current state.
    // //If it matches one of 8 possibilities (6 numbers) the wheel moved in a particular direction.
    int movement = 0;

    //Cumulative position of the wheel
    int count = 0;


    @Override
    public void init() {
        ods1 = hardwareMap.opticalDistanceSensor.get("ods1");
        ods2 = hardwareMap.opticalDistanceSensor.get("ods2");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
    }

    @Override
    public void loop() {

        raw1 = (int) (ods1.getLightDetected() * 1023);
        raw2 = (int) (ods2.getLightDetected() * 1023);

        //ODS 1 will trigger a 0 or 1 to be added to the state
        if (raw1 > 400) { //Adjust this test value to be half way between the white and black value for this sensor.
            CDI.setLED(0, true);
            state = 0;
        } else {
            CDI.setLED(0, false);
            state = 1;
        }

        //ODS 2 will trigger a 0 or 2 to be added to the state. Giving us 4 states
        if (raw2 > 400) { //Adjust this test value to be half way between the white and black value for this sensor.
            CDI.setLED(1, true);
            state = state + 0;
        } else {
            CDI.setLED(1, false);
            state = state + 2;
        }

        //Representing the previous state and current state in one number helps us avoid nested if statements
        movement = preState * 2 + state;

        //If the movement represents a value in which the previous state and current state are different, modify the count.
        if (movement == 1 || movement == 5 || movement == 8 || movement == 4)
            count++;
        if (movement == 7 || movement == 2)
            count--;

        preState = state;

        telemetry.addData("ODS1", raw1);
        telemetry.addData("ODS2", raw2);
        telemetry.addData("State", state);
        telemetry.addData("Count", count);
    }

    @Override
    public void stop() {

    }

}