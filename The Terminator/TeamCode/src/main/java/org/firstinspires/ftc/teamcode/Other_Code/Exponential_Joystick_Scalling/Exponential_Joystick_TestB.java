package org.firstinspires.ftc.teamcode.Other_Code.Exponential_Joystick_Scalling;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

/**
 * Created by Stephen McConnell on 1/30/2017.
 */
@TeleOp(name = "Exponential_Joystick", group = "Blue_Right")
@Disabled
public class Exponential_Joystick_TestB extends LinearOpMode{

    private float exp_P(float gpx) {
        float step1 = gpx *gpx; // x^2
        float step2 = step1 * gpx;                 // x^3
        float step3 = step2 * 80;                                    // 80x^3
        float step4 = step3 / 16129;                                 // 80x^3 / 127^2
        float step5 = step4 / Math.abs(gpx);       // 80x^3 / (127^2 * abs(x))
        float step6 = gpx;                    // 20x
        float step7 = step6 / Math.abs(gpx);       // 20x / abs(x)
        float outputLeft = step5 + step7;                                  // (80x^3 / (127^2 * abs(x))) + (20x / abs(x))
        return outputLeft;
    }
    private float exp_B(float gpx) {
        float output = gpx*gpx*gpx;                                // (80x^3 / (127^2 * abs(x))) + (20x / abs(x))
        return output;
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
                    float outputLeft = exp_B(-gamepad1.left_stick_y);
                    robot.leftMotor.setPower(outputLeft);
                    telemetry.addData("CalcLeft",outputLeft);

                } else if (gamepad1.right_trigger == 0){
                    robot.leftMotor.setPower(0);
                }
                if (Math.abs(gamepad1.right_stick_y) > 0) {
                    float outputRight = exp_B(-gamepad1.right_stick_y);

                    robot.rightMotor.setPower(outputRight);
                    telemetry.addData("CalcRight",outputRight);
                } else if (gamepad1.right_trigger ==0){
                    robot.rightMotor.setPower(0);
                }
                if (Math.abs(gamepad1.right_trigger) > 0) {
                    float outputRight = exp_B(-gamepad1.right_stick_y);

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

