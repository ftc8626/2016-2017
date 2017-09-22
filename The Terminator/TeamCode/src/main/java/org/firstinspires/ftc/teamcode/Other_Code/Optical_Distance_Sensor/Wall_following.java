package org.firstinspires.ftc.teamcode.Other_Code.Optical_Distance_Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Stephen McConnell on 1/8/2017.
 */

@TeleOp(name = "Follow", group = "Wall")
@Disabled
public class Wall_following extends LinearOpMode {

    OpticalDistanceSensor ODS;
    HardwareSpeedy robot = new  HardwareSpeedy();

    double odsReadingRaw;
    static double odsReadingLinear;

    @Override
    public void runOpMode() throws InterruptedException {
       // ODS = hardwareMap.opticalDistanceSensor.get("ods");
        robot.init(hardwareMap);

        robot.leftMotor.setDirection(DcMotor.Direction.REVERSE);

        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            odsReadingRaw = ODS.getRawLightDetected();
            odsReadingLinear = Math.pow(odsReadingRaw, 0.5);

            robot.leftMotor.setPower((odsReadingLinear * .333)-.0667);
            robot.rightMotor.setPower((odsReadingLinear * -.333)+1.6667);

            telemetry.addData("1 ODS linear", odsReadingLinear);
            telemetry.addData("Motor Left", robot.leftMotor.getPower());
            telemetry.addData("Motor Right", robot.rightMotor.getPower());
            telemetry.update();

        }
    }
}
