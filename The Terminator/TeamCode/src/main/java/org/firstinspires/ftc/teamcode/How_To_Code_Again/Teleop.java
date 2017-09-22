package org.firstinspires.ftc.teamcode.How_To_Code_Again;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Cash America on 8/12/2017.
 */
@TeleOp(name = "Tele_Op",group = "test")
public class Teleop extends OpMode {

    Hardwear robot = new Hardwear();
    public static final double MOTOR_POWER_OFF  = 0;
    public static final double MOTOR_POWER_FULL = 1;
    public static final double MOTOR_POWER_HALF = .5;
    @Override
    public void init(){
        robot.init(hardwareMap);
        telemetry.addData("Say", "Robot", "Initiated");
        updateTelemetry(telemetry);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
        if (gamepad1.guide){
            robot.rightMotor.setPower(MOTOR_POWER_OFF);
            robot.leftMotor.setPower(MOTOR_POWER_OFF);

            robot.servo1.setPosition(.5);
        } else {
            if gamepad1.a{
                robot.leftMotor.setPower(MOTOR_POWER_FULL);
                robot.rightMotor.setPower(MOTOR_POWER_FULL);
            } else if (gamepad1.b)
        }

    }
}
