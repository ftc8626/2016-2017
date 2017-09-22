package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Cash America on 3/21/2017.
 */
    @Autonomous(name = "Color Move Blue_Right", group = "Blue_Right")
    @Disabled
public class Color_Move_Test extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    byte [] range1Cache;
    byte [] range2Cache;

    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    static final int     COUNTS_PER_INCH         = (65);

    DcMotor leftFrontMotor   = null;
    DcMotor rightFrontMotor  = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    I2cDevice RANGE1;
    I2cDevice RANGE2;
    I2cDeviceSynch RANGE1Reader;
    I2cDeviceSynch RANGE2Reader;
    ColorSensor colorSensor1;  //Instance of ColorSensor - for reading color
    ColorSensor colorSensor2;  //Instance of ColorSensor - for reading color
    CRServo rotot = null;

    @Override
    public void runOpMode(){
        colorSensor1 = hardwareMap.colorSensor.get("color 1");
        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x3a)); //Specify I2C Address of color sensor

        colorSensor2 = hardwareMap.colorSensor.get("color 3");
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x3c)); //Specify I2C Address of color sensor
        telemetry.addData("Status", "Initialized");

        RANGE1 = hardwareMap.i2cDevice.get("range1");
        RANGE2 = hardwareMap.i2cDevice.get("range2");

        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, I2cAddr.create8bit(0x28), false);
        RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, I2cAddr.create8bit(0x30), false);

        RANGE1Reader.engage();
        RANGE2Reader.engage();

        leftBackMotor = hardwareMap.dcMotor.get("left_back_drive");
        leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        rightBackMotor = hardwareMap.dcMotor.get("right_back_drive");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        rotot = hardwareMap.crservo.get("color_pusher");
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();


        telemetry.addData("Status", "Running: " + runtime.toString());
        while (colorSensor2.blue() < 3) {
            double range1 = getRange1();
            double range2 = getRange2();
            double leftPower5 = (range2 * -.07142857) + 0.92857143;
            double rightPower5 = (range2 * .07142857) - 0.42857143;

            telemetry.addData("leftPower", leftPower5);
            telemetry.addData("rightPower", rightPower5);

            leftPower5 = Range.clip(leftPower5, 0, 1);
            rightPower5 = Range.clip(rightPower5, 0, 1);

            leftBackMotor.setPower(leftPower5);
            leftFrontMotor.setPower(leftPower5);
            rightBackMotor.setPower(rightPower5);
            rightFrontMotor.setPower(rightPower5);

            telemetry.addData("UltaSound Range 1", range1);
            telemetry.addData("UltraSonic Range 2", range2);
            telemetry.update();

            telemetry.addData("3 Red1  ", colorSensor1.red());
            telemetry.addData("5 Blue1 ", colorSensor1.blue());
            telemetry.update();
        }
        //hitting beacon
        stopMotors();
        rotot.setPower(.1);
        sleep(2000);
        rotot.setPower(-.2);
        sleep(1000);
        rotot.setPower(0);
    }
    private void stopMotors(){
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
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
