package org.firstinspires.ftc.teamcode.Speedy_Modes.Spec_Test_Speedy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Stephen McConnell on 1/21/2017.
 */
@Autonomous(name = "Color_Sensor_Test", group = "Blue_Right")
@Disabled
public class Color_Sensor_Test extends LinearOpMode {
    ColorSensor colorSensor1;  //Instance of ColorSensor - for reading color
    ColorSensor colorSensor2;  //Instance of ColorSensor - for reading color
    @Override
    public void runOpMode() throws InterruptedException {
        //the below lines set up the configuration file
        colorSensor1 = hardwareMap.colorSensor.get("color 1");
        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x3a)); //Specify I2C Address of color sensor

        colorSensor2 = hardwareMap.colorSensor.get("color 3");
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x3c)); //Specify I2C Address of color sensor

        boolean touchState = false;  //Tracks the last known state of the touch sensor
        boolean LEDState = false;     //Tracks the mode of the color sensor; Active = true, Passive = false

        waitForStart();  //Wait for the play button to be pressed

        colorSensor1.enableLed(LEDState);  //Set the mode of the LED; Active = true, Passive = false
        colorSensor2.enableLed(LEDState);
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon

        while (opModeIsActive()) {   //Main loop of program

            //The below two if() statements ensure that the mode of the color sensor is changed only once each time the touch sensor is pressed.
            //The mode of the color sensor is saved to the sensor's long term memory. Just like flash drives, the long term memory has a life time in the 10s or 100s of thousands of cycles.
            //This seems like a lot but if your program wrote to the long term memory every time though the main loop, it would shorten the life of your sensor.
            //display values
            telemetry.addData("3 Red1  ", colorSensor1.red());
            telemetry.addData("5 Blue1 ", colorSensor1.blue());

            telemetry.addData("6 Red2  ", colorSensor2.red());
            telemetry.addData("8 Blue2 ", colorSensor2.blue());
            telemetry.update();

            waitOneFullHardwareCycle();  //wait for all new data to go from the phone to the controllers and from the controllers to the phone.
        }  //End main loop of program
    }
}
