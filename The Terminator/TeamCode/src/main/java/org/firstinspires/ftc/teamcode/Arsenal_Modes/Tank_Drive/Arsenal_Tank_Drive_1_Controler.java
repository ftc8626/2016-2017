package org.firstinspires.ftc.teamcode.Arsenal_Modes.Tank_Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.Test_Robot_Init;

/**
 * Created by Stephen McConnell on 2/28/2017.
 */
@TeleOp(name="Arsenal_Tank_Drive_1_Controler", group="Blue_Right")
//@Disabled
public class Arsenal_Tank_Drive_1_Controler extends OpMode{

    /* Declare OpMode members. */
    Test_Robot_Init robot      = new Test_Robot_Init(); // use the class created to define a Pushbot's hardware
    public static final double MOTOR_FULL_STOP  = 0;
    public static final double MOTOR_HALF_FORWARD  = .4
            ;
    public static final double MOTOR_HALF_REVERSE  = -.4;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "HEY WHAT ARE YOU DOING?", "DON'T TOUCH ME THERE");    //
        updateTelemetry(telemetry);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    @Override
    public void init_loop() {}
    @Override
    public void start() {}
    @Override
    public void loop() {
        if (gamepad1.guide){
            robot.rightShooterMotor.setPower(0);
            robot.leftShooterMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.intakeMotor.setPower(0);
        }else {
            double left;
            double right;

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            right = -gamepad1.left_stick_y;
            left = -gamepad1.right_stick_y;
            robot.leftBackMotor.setPower(left);
            robot.leftFrontMotor.setPower(left);
            robot.rightBackMotor.setPower(right);
            robot.rightFrontMotor.setPower(right);

            if (gamepad1.y) {
                robot.intakeMotor.setPower(MOTOR_HALF_REVERSE);
            } else if (gamepad1.a) {
                robot.intakeMotor.setPower(MOTOR_HALF_FORWARD);
            }

            if (gamepad1.dpad_up) {
                robot.leftShooterMotor.setPower(.9);
                  robot.rightShooterMotor.setPower(.9);
            } else if (gamepad1.dpad_down) {
                robot.leftShooterMotor.setPower(-1);
                robot.rightShooterMotor.setPower(-1);
            }

            if (gamepad1.x) {
                robot.shooterFlicker.setPosition(.5);
            } else if (gamepad1.b){
                robot.shooterFlicker.setPosition(.24);
            }else {
                robot.shooterFlicker.setPosition(.35);
            }

            if (gamepad1.left_trigger > 0) {
                robot.flick.setPosition(0);
            } else {
                robot.flick.setPosition(.6);
            }
//            if (gamepad1.left_trigger>0){
//                robot.rotot.setPower(-1);
//            }else if (gamepad1.right_trigger>0){
//                robot.rotot.setPower(1);
//            }else{
//                robot.rotot.setPower(0);
//            }
            if (gamepad1.right_bumper) {
                robot.rotot.setPower(.2);
            }else if (gamepad1.left_bumper){
                robot.rotot.setPower (-.2);
            }else {
                robot.rotot.setPower(0);
            }

            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0 ){
                robot.liftMotor.setPower(1);
                robot.capArm.setPosition(0);
            } else if (gamepad1.back){
                robot.liftMotor.setPower(-.2);
                robot.capArm.setPosition(.7);
            }else {
                robot.liftMotor.setPower(0);
            }
        }
      }
    }
