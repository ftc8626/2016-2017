package org.firstinspires.ftc.teamcode.Speedy_Modes.Spec_Test_Speedy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;
import org.firstinspires.ftc.teamcode.Other_Code.Extra.Test_Robot_Init;

/**
 * Created by Cash America on 1/21/2017.
 */
@Autonomous(name = "Drive_Test", group = "Blue_Right")
@Disabled
public class Drive_Test extends LinearOpMode {
    Test_Robot_Init robot = new Test_Robot_Init();
    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        robot.leftBackMotor.setPower(.3);
        robot.rightBackMotor.setPower(.3);
        robot.leftFrontMotor.setPowerFloat();
        robot.rightFrontMotor.setPowerFloat();
        sleep(2000);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
    }
}
