package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 1/27/2017.
 */

@TeleOp(name = "Speed Blue_Right", group = "Blue_Right")
@Disabled
public class Max_Speed_Test extends LinearOpMode {
    HardwareSpeedy robot = new HardwareSpeedy();
    @Override
    public void runOpMode(){
        robot.leftShooterMotor.setMaxSpeed(1);
        robot.rightShooterMotor.setMaxSpeed(1);
    }
}
