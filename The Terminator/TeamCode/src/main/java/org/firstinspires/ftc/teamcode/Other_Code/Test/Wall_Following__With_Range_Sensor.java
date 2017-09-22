package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 2/28/2017.
 */
@TeleOp(name = "Wall_Following_With_Range_Sensor", group = "Range Sensor")
@Disabled
public class Wall_Following__With_Range_Sensor extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareSpeedy robot = new  HardwareSpeedy();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
//    String str = new String(range1Cache);
//    byte b1 = Byte.parseByte(str);
//    double x = b1;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    @Override
    public void runOpMode() throws InterruptedException {
        double rangeAsDouble;
        double test;
        double leftPower;
        double rightPower;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        robot.leftMotor.setDirection(DcMotor.Direction.REVERSE);

        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

            rangeAsDouble = range1Cache[0] & 0xFF;
            test = Range.clip(rangeAsDouble,0,1);
            leftPower =((test *-(1/14) )+(13/14));
            rightPower =((test *(1/14) )-(3/7));

//            robot.leftMotor.setPower();
//            robot.rightMotor.setPower();

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("Ultra Sonic Double", rangeAsDouble);
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Righ Power", rightPower);
//            telemetry.addData("Motor Left", robot.leftMotor.getPower());
//            telemetry.addData("Motor Right", robot.rightMotor.getPower());
            telemetry.update();

            idle();
        }



    }

//    public double getRange() {
//
//        double range = 0;
//
//        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//
//        byte byteRange = range1Cache[0]; // & 0xFF
//
//            telemetry.addData("Range Sensor Follow", (range1Cache[0] & 0xFF));
//            range = byteRange; // uses an implicit cast from byte to double
//            telemetry.addData("Range (as double)", range);
//        telemetry.update();
//        return range;
//    }

}

