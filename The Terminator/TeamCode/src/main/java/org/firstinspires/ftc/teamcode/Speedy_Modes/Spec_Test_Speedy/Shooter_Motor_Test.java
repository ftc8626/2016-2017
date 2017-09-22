package org.firstinspires.ftc.teamcode.Speedy_Modes.Spec_Test_Speedy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 1/21/2017.
 */
@Autonomous (name = "Shooter_Test", group = "Blue_Right")
@Disabled
public class Shooter_Motor_Test extends LinearOpMode{
    HardwareSpeedy robot = new HardwareSpeedy();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.leftShooterMotor.setPower(1);
        robot.rightShooterMotor.setPower(1);
        sleep(3000);
        robot.leftShooterMotor.setPower(0);
        robot.rightShooterMotor.setPower(0);
        sleep(3000);
        robot.leftShooterMotor.setPower(-1);
        robot.rightShooterMotor.setPower(-1);
        sleep(3000);
        robot.leftShooterMotor.setPower(0);
        robot.rightShooterMotor.setPower(0);
    }


    }
