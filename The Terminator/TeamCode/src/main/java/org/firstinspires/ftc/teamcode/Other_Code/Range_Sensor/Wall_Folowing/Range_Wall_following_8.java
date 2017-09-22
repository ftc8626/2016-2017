package org.firstinspires.ftc.teamcode.Other_Code.Range_Sensor.Wall_Folowing;

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

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedremake;

/**
 * Created by Stephen McConnell on 1/8/2017.
 */

@TeleOp(name = "Follow .8", group = "Wall")
@Disabled
public class Range_Wall_following_8 extends LinearOpMode {

//    OpticalDistanceSensor ODS;
    HardwareSpeedremake robot = new HardwareSpeedremake();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    static final int     COUNTS_PER_INCH         = (65);


    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);

        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        // TODO: Why is the following statement needed?
        runtime.reset();

        if (gamepad1.guide){
            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(0);
        } else {
            while (opModeIsActive()) {
                double range = getRange();
                double leftPower8 = (range * -.11428571) + 1.48571429;
                double rightPower8 = (range * .11428571) - .68571429;

                telemetry.addData("leftPower", leftPower8);
                telemetry.addData("rightPower", rightPower8);

                leftPower8 = Range.clip(leftPower8, 0, 1);
                rightPower8 = Range.clip(rightPower8, 0, 1);

                robot.leftMotor.setPower(leftPower8);
                robot.rightMotor.setPower(rightPower8);

                telemetry.addData("UltaSound Range", range);
                telemetry.addData("Motor Left", robot.leftMotor.getPower());
                telemetry.addData("Motor Right", robot.rightMotor.getPower());

                telemetry.update();
            }
        }
    }

    private double getRange() {
        double ultraSonicRange = 0;

        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        byte byteUltraSonicRange = range1Cache[0];

        // The following line uses implicit type casting don't by the JVM
        // bytes -> short -> int -> long -> double, etc...
        ultraSonicRange = byteUltraSonicRange;



        telemetry.addData("Ultra Sonic (byte)", byteUltraSonicRange & 0xFF);
        telemetry.addData("Ultra Sonic (double)", ultraSonicRange);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        return ultraSonicRange;
    }
}
