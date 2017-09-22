package org.firstinspires.ftc.teamcode.Speedy_Modes.Spec_Test_Speedy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 1/21/2017.
 */
@Autonomous(name = "Color_Motor", group = "Blue_Right")
 @Disabled
public class Color_Motor_Test extends LinearOpMode {
    HardwareSpeedy robot = new HardwareSpeedy();
    @Override
    public void runOpMode() throws InterruptedException {
     robot.init(hardwareMap);

        robot.colorMotor.setPower(.1);
        sleep(500);
        robot.colorMotor.setPower(-.1);
        sleep(500);
        robot.colorMotor.setPower(0);
        sleep(2000);
        robot.colorMotor.setPower(-.1);
        sleep(500);
        robot.colorMotor.setPower(.1);
        sleep(500);
        robot.colorMotor.setPower(0);

    }
}
