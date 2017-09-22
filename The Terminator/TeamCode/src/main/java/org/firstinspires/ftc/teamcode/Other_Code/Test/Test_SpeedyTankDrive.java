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
package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;

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

@TeleOp(name="Blue_Right Teleop Tank", group="Speedy")
@Disabled
public class Test_SpeedyTankDrive extends OpMode {

    /* Declare OpMode members. */
    HardwareSpeedy robot      = new HardwareSpeedy(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
                                                         // servoGate positions
    double          clawOffset  = .5;                  // Servo mid position

    //final double    CLAW_SPEED  = 1 ;                 // sets rate to move servo

    private final double SHOOTER_OUT_SPEED = .2;
    private final double SHOOTER_IN_SPEED = -.1;
    double SHOOTER_MOTOR_POWER_FACTOR = .8;

    public static final double BUMPER_UP = 1;
    public static final double BUMPER_DOWN = .5;

    public static final double BUTTON_PUSHER_RETRACTED = 0;
    public static final double BUTTON_PUSHER_EXTENDED  = 1;

    public static final double MOTOR_FULL_STOP  = 0;
    public static final double MOTOR_HALF_FORWARD  = .4;
    public static final double MOTOR_HALF_REVERSE  = -.4;
    public static final double MOTOR_FULL_FORWARD  = 1;
    public static final double MOTOR_FULL_REVERSE  = -1;

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

//        robot.shooterMotor.setPower(1);
//        robot.intakeMotor.setPower(1);
       /* if (robot.LEDAState){
            robot.colorAreader.write8(3,0);
        }
        else {
            robot.colorAreader.write8(3,1);
        }*/
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

   /*     robot.colorAcache = robot.colorAreader.read(0x04,1);
        telemetry.addData("Color Readings",robot.colorAcache[0] & 0xFF);
*/
        if (gamepad1.guide && gamepad2.guide){
            // In case of an emergency!
            stopAllMotors();
            moveServosToStartingPositions();
        }

        /*else if (gamepad2.guide && gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0) {

            robot.rightShooterMotor.setPower(0);
            robot.leftShooterMotor.setPower(0);
            robot.intakeMotor.setPower(0);

        }*/else if (gamepad1.guide){
            stopAllMotors();
            moveServosToStartingPositions();
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
                robot.rightShooterMotor.setPower(MOTOR_FULL_REVERSE);
                robot.leftShooterMotor.setPower(MOTOR_FULL_REVERSE);
            } else if(gamepad1.dpad_down) {
                robot.rightShooterMotor.setPower(.4);
                robot.leftShooterMotor.setPower(.4);
            }
            else if (gamepad1.right_trigger > 0 && gamepad1.x) {
                robot.rightShooterMotor.setPower(.2);
                robot.leftShooterMotor.setPower(.2);
            }
//
//            if (gamepad1.right_trigger > 0 && gamepad1.a)
//                // Extend left button pusher to maximum position.
//                robot.leftColorArm.setPosition(BUTTON_PUSHER_EXTENDED);
//            else if (gamepad1.left_trigger > 0 && gamepad1.a)
//                // Retract left button pusher to minimum position.
//                robot.leftColorArm.setPosition(BUTTON_PUSHER_RETRACTED);
//
//            if (gamepad1.right_trigger > 0 && gamepad1.b)
//                robot.rightColorArm.setPosition(BUTTON_PUSHER_RETRACTED);
//            else if (gamepad1.left_trigger > 0 && gamepad1.b)
//                robot.rightColorArm.setPosition(BUTTON_PUSHER_EXTENDED);

            //********************
            //shooter speed
            //********************

             double shooterOutSpeed = SHOOTER_OUT_SPEED * SHOOTER_MOTOR_POWER_FACTOR;
             double shooterInSpeed  = SHOOTER_IN_SPEED * SHOOTER_MOTOR_POWER_FACTOR;

             shooterOutSpeed = Range.clip(shooterOutSpeed, -1, 1);
             shooterInSpeed = Range.clip(shooterInSpeed, -1, 1);

            // lift controls
            if (gamepad1.y) {
                robot.intakeMotor.setPower(MOTOR_HALF_REVERSE);
            } else if (gamepad1.a) {
                robot.intakeMotor.setPower(MOTOR_HALF_FORWARD);
            } else {
                robot.intakeMotor.setPower(MOTOR_FULL_STOP);
              }

            if (gamepad1.dpad_right) {
                DcMotor tempMotor = robot.leftMotor;
                robot.leftMotor = robot.rightMotor;
                robot.rightMotor = tempMotor;
//                robot.leftMotor = hwMap.dcMotor.get("left_drive");
//                robot.rightMotor = hwMap.dcMotor.get("right_drive");
//                robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);
//                robot.leftMotor.setDirection(DcMotor.Direction.FORWARD);
            }


        //shooter controls
//        if (gamepad2.right_trigger > 0 && gamepad2.a) {
//            robot.rightShooterMotor.setPower(shooterOutSpeed);
//            robot.leftShooterMotor.setPower(shooterOutSpeed);
//        }else if (gamepad2.left_trigger > 0 && gamepad2.a) {
//            robot.rightShooterMotor.setPower(shooterInSpeed);
//            robot.leftShooterMotor.setPower(shooterInSpeed);
//        }else if (gamepad2.guide) {
//            robot.rightShooterMotor.setPower(MOTOR_FULL_STOP);
//            robot.leftShooterMotor.setPower(MOTOR_FULL_STOP);
//        }

        //permanent set
//        if (gamepad2.guide && gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
//            while (gamepad1.right_trigger >= 0 && gamepad1.left_trigger >= 0) {
//                robot.shooterMotor.setPower(1);
//                robot.intakeMotor.setPower(1);
//            }
//        }else{
//            controle();
//        }

            // Send telemetry message to signify robot running;
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            updateTelemetry(telemetry);
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
//        robot.rightColorArm.setPosition(COLOR_SERVO_START);
//        robot.leftColorArm.setPosition(COLOR_SERVO_START);
//        robot.bumper.setPosition(MAX_POSITION);
    }

}
