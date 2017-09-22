package org.firstinspires.ftc.teamcode.Speedy_Modes.Spec_Test_Speedy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Cash America on 1/21/2017.
 */
@Autonomous (name = "Bumper_Test", group = "Blue_Right")
@Disabled
public class Bumper_Test extends LinearOpMode {
    HardwareSpeedy robot = new HardwareSpeedy();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.bumper.setPosition(1);
        sleep(2000);
        robot.bumper.setPosition(.5);

    }
}
