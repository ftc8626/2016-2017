package org.firstinspires.ftc.teamcode.Other_Code.Optical_Distance_Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 1/9/2017.
 */

@TeleOp(name = "Turn Blue_Right", group = "ods")
@Disabled
public class Turn_Test  extends LinearOpMode {
    OpticalDistanceSensor ODS;
    HardwareSpeedy robot = new HardwareSpeedy();

    double odsReadingRaw;
    static double odsReadingsLinear;

    @Override
    public void runOpMode() throws InterruptedException{
       // ODS = hardwareMap.opticalDistanceSensor.get("ods");
        robot.init(hardwareMap);

        robot.leftMotor.setDirection(DcMotor.Direction.REVERSE);

        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            odsReadingRaw = ODS.getRawLightDetected()/5;
            odsReadingsLinear = Math.pow(odsReadingRaw, 0.5);
        }
    }
}
