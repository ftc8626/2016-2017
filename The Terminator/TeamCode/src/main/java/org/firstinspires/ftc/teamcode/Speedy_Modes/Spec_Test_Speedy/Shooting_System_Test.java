package org.firstinspires.ftc.teamcode.Speedy_Modes.Spec_Test_Speedy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 1/21/2017.
 */
@Autonomous(name = "Shootin_System_Test",group = "Blue_Right")
@Disabled
public class Shooting_System_Test extends LinearOpMode {
    HardwareSpeedy robot = new HardwareSpeedy();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.leftShooterMotor.setPower(Math.abs(.65));
        robot.rightShooterMotor.setPower(Math.abs(.65));
        Thread.sleep(500);
        robot.intakeMotor.setPower(-.5);
        Thread.sleep(2 * 1000);
        robot.intakeMotor.setPower(0);
        robot.leftShooterMotor.setPower(Math.abs(0));
        robot.rightShooterMotor.setPower(Math.abs(0));
    }
}
