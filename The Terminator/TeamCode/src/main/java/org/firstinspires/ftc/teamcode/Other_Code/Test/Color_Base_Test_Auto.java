/*
package org.firstinspires.ftc.teamcode.Other_Code.Test;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Stephen McConnell on 1/9/2017.
 */
/*
@Autonomous(name = "Color_Base_Test", group = "color")
@Disabled
public class Color_Base_Test_Auto extends LinearOpMode {
    HardwareSpeedy robot = new HardwareSpeedy();
    DeviceInterfaceModule CDI;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_INCH = (65);
    static final double DRIVE_SPEED = 0.6;

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d : %7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        boolean LEDAState = true;

        waitForStart();
        robot.color_Sensor_A.enableLed(LEDAState);

//        float hsvValues []= {0, 0, 0};

        encoderDriverColor(DRIVE_SPEED, 10000, 10000, 5.0);


//        while (!(isStarted() || isStopRequested())) {
//            Color.RGBToHSV(robot.color_Sensor_A.red() * 8, robot.color_Sensor_A.blue() * 8, robot.color_Sensor_A.blue() * 8, hsvValues);
//
//            telemetry.addData("Clear", robot.color_Sensor_A.alpha());
//            telemetry.addData("Red", robot.color_Sensor_A.red());
//            telemetry.addData("Green", robot.color_Sensor_A.green());
//            telemetry.addData("Blue", robot.color_Sensor_A.blue());
//            telemetry.addData("Hue", hsvValues[0]);
//            telemetry.update();
//
//            if (robot.color_Sensor_A.red() > robot.color_Sensor_A.blue() && robot.color_Sensor_A.red() > robot.color_Sensor_A.green()) {
//                CDI.setLED(1, true);
//                CDI.setLED(0, false);
//            } else if (robot.color_Sensor_A.blue() > robot.color_Sensor_A.red() && robot.color_Sensor_A.blue() > robot.color_Sensor_A.green()) {
//                CDI.setLED(1, false);
//                CDI.setLED(0, true);
//            } else if (robot.color_Sensor_A.green() == robot.color_Sensor_A.red() && robot.color_Sensor_A.green() == robot.color_Sensor_A.blue() && robot.color_Sensor_A.green() > 0) {
//                CDI.setLED(1, true);
//                CDI.setLED(0, true);
//            } else {
//                CDI.setLED(1, false);
//                CDI.setLED(0, false);
//            }
//        }

   //     encoderDriver(DRIVE_SPEED, 100000000, 100000000, 5.0);


    }

    public void encoderDriverColor(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                float hsvValues []= {0, 0, 0};

                Color.RGBToHSV(robot.color_Sensor_A.red() * 8, robot.color_Sensor_A.blue() * 8, robot.color_Sensor_A.blue() * 8, hsvValues);

                telemetry.addData("Clear", robot.color_Sensor_A.alpha());
                telemetry.addData("Red", robot.color_Sensor_A.red());
                telemetry.addData("Green", robot.color_Sensor_A.green());
                telemetry.addData("Blue", robot.color_Sensor_A.blue());
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();

                if (robot.color_Sensor_A.red() > robot.color_Sensor_A.blue() && robot.color_Sensor_A.red() > robot.color_Sensor_A.green()) {
                   CDI.setLED(1, true);
                   CDI.setLED(0, false);
                } else if (robot.color_Sensor_A.blue() > robot.color_Sensor_A.red() && robot.color_Sensor_A.blue() > robot.color_Sensor_A.green()) {
                   CDI.setLED(1, false);
                   CDI.setLED(0, true);
                } else if (robot.color_Sensor_A.green() == robot.color_Sensor_A.red() && robot.color_Sensor_A.green() == robot.color_Sensor_A.blue() && robot.color_Sensor_A.green() > 0) {
                    robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(0);

                    CDI.setLED(1, true);
                    CDI.setLED(1, true);
                } else {
                    CDI.setLED(1, false);
                    CDI.setLED(0, false);
                }

                telemetry.addData("path1", "Running to %7d: %7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d : %7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
*/