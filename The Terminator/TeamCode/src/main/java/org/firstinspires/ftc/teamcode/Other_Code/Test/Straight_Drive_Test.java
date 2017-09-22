package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by James Coulter on 3/18/2017.
 */
@TeleOp(name = "Better_Joystick", group = "Blue_Right")
@Disabled
public class Straight_Drive_Test extends LinearOpMode {

    private float sensitive(float gpx) {
        float minToMove = 0.14f;
        float scaleTheRestOfTheWay = 1.0f - minToMove;
        float scaledPower = 0.0f;

        if(Math.abs(gpx)< 0.05)
            scaledPower = 0.0f;
        else if (gpx > 0.0)
            scaledPower = minToMove + scaleTheRestOfTheWay * gpx;
        else if (gpx < 0.0)
            scaledPower = -minToMove + scaleTheRestOfTheWay * gpx;
        return scaledPower;
    }
    HardwareSpeedy robot = new HardwareSpeedy();
    @Override
    public void runOpMode () {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) //infinite loop
        {

            if (Math.abs(gamepad1.left_stick_y) > 0) //threshold of 5
            {
                float outputLeft = sensitive(-gamepad1.left_stick_y);
                robot.leftMotor.setPower(outputLeft);
                telemetry.addData("CalcLeft",outputLeft);

            } else if (gamepad1.right_trigger == 0){
                robot.leftMotor.setPower(0);
            }
            if (Math.abs(gamepad1.right_stick_y) > 0) {
                float outputRight = sensitive(-gamepad1.right_stick_y);

                robot.rightMotor.setPower(outputRight);
                telemetry.addData("CalcRight",outputRight);
            } else if (gamepad1.right_trigger ==0){
                robot.rightMotor.setPower(0);
            }
            if (Math.abs(gamepad1.right_trigger) > 0) {
                float outputRight = sensitive(-gamepad1.right_stick_y);

                robot.rightMotor.setPower(outputRight);
                robot.leftMotor.setPower(outputRight);
                telemetry.addData("Input Trigger", gamepad1.right_trigger);
            } else if (gamepad1.right_stick_y ==0 && gamepad1.left_stick_y ==0){
                robot.rightMotor.setPower(0);
                robot.leftMotor.setPower(0);
            }
            telemetry.addData("Input Left", -gamepad1.left_stick_y);
            telemetry.addData("Input Right", -gamepad1.right_stick_y);
            telemetry.addData("OutputLeft", robot.leftMotor.getPower());
            telemetry.addData("OutputRight", robot.rightMotor.getPower());
            telemetry.update();
        }
    }

}
