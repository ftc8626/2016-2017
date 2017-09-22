package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 1/8/2017.
 */
@Autonomous(name = "Base_Test_Auto_2", group = "Blue_Right")
@Disabled
public class Base_Test_Auto_2 extends LinearOpMode {
    HardwareSpeedy robot = new HardwareSpeedy();

    OpticalDistanceSensor lightSensor;

    static final double        WHITE_THRESHOLD = 0.2;
    static final double        APPROACH_SPEED   = 0.5;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lightSensor = hardwareMap.opticalDistanceSensor.get("ods");

        lightSensor.enableLed(true);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {

            telemetry.addData("light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle();
        }
        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);

        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)){

            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
        }
        robot.leftMotor.setPower(0);
        robot.leftMotor.setPower(0);
    }
}
