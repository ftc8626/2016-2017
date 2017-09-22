package org.firstinspires.ftc.teamcode.Speedy_Modes.Spec_Test_Speedy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 1/21/2017.
 */
@Autonomous(name = "Sweeper_Motor_Test",group = "Blue_Right")
@Disabled
public class Sweeper_Motor_Test  extends LinearOpMode{
    HardwareSpeedy robot = new HardwareSpeedy();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.intakeMotor.setPower(-.5);
        Thread.sleep(2 * 1000);
        robot.intakeMotor.setPower(0);
        sleep(3000);
        robot.intakeMotor.setPower(.5);
        Thread.sleep(2 * 1000);
        robot.intakeMotor.setPower(0);


    }
}
