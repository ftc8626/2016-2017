package org.firstinspires.ftc.teamcode.Arsenal_Modes.Spec_Test_Arsenal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Cash America on 3/19/2017.
 */
@Autonomous(name = "Range Sensor Blue_Right Arsenal",group = "Arsenal Blue_Right")
//@Disabled
public class Range_Sensor_Test extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    byte[] range2Cache;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDevice RANGE2;
    public I2cDeviceSynch RANGE1Reader;
    public I2cDeviceSynch RANGE2Reader;
    DeviceInterfaceModule CDI;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RANGE1 = hardwareMap.i2cDevice.get("range1");
        RANGE2 = hardwareMap.i2cDevice.get("range2");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, I2cAddr.create8bit(0x28), false);
        RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, I2cAddr.create8bit(0x30), false);

        RANGE1Reader.engage();
        RANGE2Reader.engage();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double range1 = getRange1();
            double range2 = getRange2();
            telemetry.addData("UltaSound Range 1", range1);
            telemetry.addData("UltraSonic Range 2", range2);
            telemetry.update();

            if (range1 < 30){
                CDI.setLED(1, true);       //Red ON
                CDI.setLED(0, false);      //Blue OFF
            } else if (range2 < 30) {
                CDI.setLED(1, false);       //Red OFF
                CDI.setLED(0, true);      //Blue ON
            }else{
                CDI.setLED(1, false);       //Red OFF
                CDI.setLED(0, false);      //Blue OFF
            }
        }
    }
    private double getRange1() {
        double ultraSonicRange1 = 0;

        range1Cache = RANGE1Reader.read(0x04,2);

        byte byteUltraSonicRange1 = range1Cache[0];

        // The following line uses implicit type casting don't by the JVM
        // bytes -> short -> int -> long -> double, etc...
        ultraSonicRange1 = byteUltraSonicRange1;



//        telemetry.addData("Ultra Sonic (byte)", byteUltraSonicRange & 0xFF);
//        telemetry.addData("Ultra Sonic (double)", ultraSonicRange);
//
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.update();

        return ultraSonicRange1;
    }

    private double getRange2() {
        double ultraSonicRange2 = 0;

        range2Cache = RANGE2Reader.read(0x04,2);

        byte byteUltraSonicRange2 = range2Cache[0];

        // The following line uses implicit type casting don't by the JVM
        // bytes -> short -> int -> long -> double, etc...
        ultraSonicRange2 = byteUltraSonicRange2;



//        telemetry.addData("Ultra Sonic (byte)", byteUltraSonicRange & 0xFF);
//        telemetry.addData("Ultra Sonic (double)", ultraSonicRange);
//
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.update();

        return ultraSonicRange2;
    }
}
