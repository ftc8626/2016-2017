/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Speedy_Modes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;
//import static org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy.COLOR_SERVO_START;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Speedy: Teleop Tank Drive", group="Speedy")
@Disabled
public class SpeedyTankDrive extends OpMode {

    /* Declare OpMode members. */
    HardwareSpeedy robot      = new HardwareSpeedy(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
                                                         // servoGate positions



    public static final double BUMPER_UP = 1;
    public static final double BUMPER_DOWN = .5;

    public static final double BUTTON_PUSHER_RETRACTED = 0;
    public static final double BUTTON_PUSHER_EXTENDED  = 1;

    public static final double MOTOR_FULL_STOP  = 0;
    public static final double MOTOR_HALF_FORWARD  = .4;
    public static final double MOTOR_HALF_REVERSE  = -.4;
    public static final double MOTOR_FULL_REVERSE  = -.55;
    public static final double SERVO_OUT = 1;
    public static final double  SERVO_IN = -1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "HEY WHAT ARE YOU DOING?", "DON'T TOUCH ME THERE");    //
        updateTelemetry(telemetry);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        robot.rightShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.leftShooterMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.guide){
            stopAllMotors();
        } else {
            double left;
            double right;

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);

            // Use gamepad left & right Bumper to open and close
            if (gamepad1.b)
                // Lift bumper up
                robot.bumper.setPosition(BUMPER_UP);
            else if (gamepad1.x)
                // Lower bumper
                robot.bumper.setPosition(BUMPER_DOWN);
//
            if (gamepad1.dpad_up) {
                robot.rightShooterMotor.setPower(Math.abs(-.5));
                robot.leftShooterMotor.setPower(Math.abs(-.5));
            } else if(gamepad1.dpad_down) {
                robot.rightShooterMotor.setPower(-.4);
                robot.leftShooterMotor.setPower(-.4);
            }

            if(gamepad1.right_bumper){
                robot.leftShooterMotor.setPower(Math.abs(.65));
                robot.rightShooterMotor.setPower(Math.abs(.65));
            }
            else if (gamepad1.left_bumper){
                robot.leftShooterMotor.setPower(Math.abs(.55));
                robot.rightShooterMotor.setPower(Math.abs(.55));
            }

            if (gamepad1.right_trigger > 0){
                robot.colorMotor.setPower(.1);
            }
            else if (gamepad1.left_trigger > 0){
                robot.colorMotor.setPower(-.1);
            }
            else {
                robot.colorMotor.setPower(0);
            }

            if (gamepad1.start){
                robot.ballLeft.setPosition(.9);
                robot.ballRight.setPosition(.1);
            }
            else if (gamepad1.back){
                robot.ballLeft.setPosition(.5);
                robot.ballRight.setPosition(.5);
            }
//
//            if (gamepad1.dpad_left){
//             robot.ballLeft.setPosition(.5);
//            }
//            else if (gamepad1.dpad_right){
//                robot.ballRight.setPosition(.5);
//            }




            // lift controls
            if (gamepad1.y) {
                robot.intakeMotor.setPower(MOTOR_HALF_REVERSE);
            } else if (gamepad1.a) {
                robot.intakeMotor.setPower(MOTOR_HALF_FORWARD);
            } else {
                robot.intakeMotor.setPower(MOTOR_FULL_STOP);
              }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.leftMotor.setPower(MOTOR_FULL_STOP);
        robot.rightMotor.setPower(MOTOR_FULL_STOP);
        robot.intakeMotor.setPower(MOTOR_FULL_STOP);
        robot.rightShooterMotor.setPower(MOTOR_FULL_STOP);
        robot.leftShooterMotor.setPower(MOTOR_FULL_STOP);
    }

    private void stopAllMotors() {

        robot.leftMotor.setPower(MOTOR_FULL_STOP);
        robot.rightMotor.setPower(MOTOR_FULL_STOP);
        robot.intakeMotor.setPower(MOTOR_FULL_STOP);
        robot.rightShooterMotor.setPower(MOTOR_FULL_STOP);
        robot.leftShooterMotor.setPower(MOTOR_FULL_STOP);
    }

    private void moveServosToStartingPositions() {
//
   /*     robot.rightColorArm.setPosition(COLOR_SERVO_START);
        robot.leftColorArm.setPosition(COLOR_SERVO_START);*/
        robot.bumper.setPosition(MAX_POSITION);
    }

}
