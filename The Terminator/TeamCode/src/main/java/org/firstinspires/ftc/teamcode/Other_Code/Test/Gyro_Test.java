package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 2/22/2017.
 */
@Autonomous(name = "Auto_Code_Move", group = "Gyro")
@Disabled
public class Gyro_Test extends LinearOpMode{
    HardwareSpeedy robot   = new HardwareSpeedy();
    private ElapsedTime runtime = new ElapsedTime();

    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro mrGyro;

    int zAccumulated;
    int heading;
    int xVal, yVal, zVal;

    @Override
  public void runOpMode()throws InterruptedException {
        robot.init(hardwareMap);
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;
        mrGyro.calibrate();
        while (mrGyro.isCalibrating()){
        }
        waitForStart();
        zAccumulated = mrGyro.getIntegratedZValue();
        heading = 360 - mrGyro.getHeading();
        if (heading == 360){
            heading = 0;
        }
        xVal = mrGyro.rawX() / 128;
        yVal = mrGyro.rawX() / 128;
        zVal = mrGyro.rawZ() / 128;

        while (opModeIsActive()) {
            telemetry.addData("heading", String.format("%03d", heading));
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.addData("X", String.format("%03d", xVal));
            telemetry.addData("Y", String.format("%03d", yVal));
            telemetry.addData("Z", String.format("%03d", zVal));
            telemetry.update();
        }
    }
}
