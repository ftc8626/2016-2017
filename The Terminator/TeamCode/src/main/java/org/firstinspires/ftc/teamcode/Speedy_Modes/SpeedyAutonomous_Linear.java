package org.firstinspires.ftc.teamcode.Speedy_Modes;/*
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.Category;
import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;
import org.firstinspires.ftc.teamcode.Other_Code.Extra.MenuChoices;
import org.firstinspires.ftc.teamcode.Other_Code.Extra.OptionMenu;
import org.firstinspires.ftc.teamcode.Other_Code.Extra.SingleSelectCategory;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Speedy: Autonomous", group="Speedy")
@Disabled
public class SpeedyAutonomous_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSpeedy robot   = new HardwareSpeedy();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
   // static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
   //                                                  (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DEGREES              = (65 * ((22*Math.PI)/360));
    static final double     COUNTS_PER_INCH         = (65);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.4;

    public final String ALLIANCE = "ALLIANCE";
    public final String ALLIANCE_RED = "RED";
    public final String ALLIANCE_BLUE = "BLUE";
    public final String STARTING_POSITION = "STARTING POSITION";
    public final String STARTING_POSITION_LEFT = "LEFT";
    public final String STARTING_POSITION_RIGHT = "RIGHT";
    public final String START_DELAY = "START DELAY";
    public final String SHOULD_PARK = "SHOULD PARK";
    public final String SHOULD_PARK_TRUE = "YES";
    public final String SHOULD_PARK_FALSE = "NO";

    ColorSensor colorSensor1;  //Instance of ColorSensor - for reading color
    ColorSensor colorSensor2;  //Instance of ColorSensor - for reading color

    private OptionMenu menu;
    @Override
    public void runOpMode() throws InterruptedException {



        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

       try {
           initializeRobot();

           waitForStart();

           makeSomeMoves();

           shutDownRobot();

       } catch (Exception ex) {
           telemetry.addData("Error", ex.getMessage());
       }


    }
    private void initializeRobot () throws InterruptedException, IllegalStateException, Exception{
        robot.init(hardwareMap);
        InitializeMenu();

        robot.leftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rightMotor.setDirection(DcMotor.Direction.FORWARD);

        colorSensor1 = hardwareMap.colorSensor.get("color 1");
        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x3a)); //Specify I2C Address of color sensor

        colorSensor2 = hardwareMap.colorSensor.get("color 3");
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x3c)); //Specify I2C Address of color sensor


    }

    private void InitializeMenu() throws InterruptedException{
        OptionMenu.Builder builder = new OptionMenu.Builder(hardwareMap.appContext);

        //setup a Alliance Category
        SingleSelectCategory alliance = new SingleSelectCategory(ALLIANCE);
        alliance.addOption(ALLIANCE_RED);
        alliance.addOption(ALLIANCE_BLUE);
        builder.addCategory(alliance);

        //Setup a strting position Category
        SingleSelectCategory startPosition = new SingleSelectCategory(STARTING_POSITION);
        startPosition.addOption (STARTING_POSITION_LEFT);
        startPosition.addOption (STARTING_POSITION_RIGHT);
        builder.addCategory(startPosition);

        //Setup Whether or not to park in te floor goal zone
        SingleSelectCategory shouldPark = new SingleSelectCategory(SHOULD_PARK);
        shouldPark.addOption(SHOULD_PARK_TRUE);
        shouldPark.addOption(SHOULD_PARK_FALSE);
        builder.addCategory(shouldPark);

        //setup a start delay category
        SingleSelectCategory startDelay = new SingleSelectCategory(START_DELAY);
        startDelay.addOption("0");
        startDelay.addOption("1");
        startDelay.addOption("2");
        startDelay.addOption("3");
        startDelay.addOption("4");
        startDelay.addOption("5");
        startDelay.addOption("6");
        startDelay.addOption("7");
        startDelay.addOption("8");
        startDelay.addOption("9");
        startDelay.addOption("10");
        startDelay.addOption("11");
        startDelay.addOption("12");
        startDelay.addOption("13");
        startDelay.addOption("14");
        startDelay.addOption("15");
        builder.addCategory(startDelay);

        //Create menu
        menu = builder.create();

        //Display menu
        menu.show();

    }
    private MenuChoices getMenuChoices() throws Exception {
        MenuChoices menuChoices = new MenuChoices();

        if (menu.isEmpty()) {
            throw new Exception("please restart and select menu items");
        }

        for (Category c : menu.getCategories()) {

            String categoryName = c.getName();
            telemetry.addData("menu categoryName: ", categoryName);
            telemetry.addData("menu selectedOption: ", menu.selectedOption(categoryName));

            if (categoryName == ALLIANCE)
                menuChoices.setAlliance(menu.selectedOption(categoryName));
            else if (categoryName == STARTING_POSITION)
                menuChoices.setStartingPosition(menu.selectedOption(categoryName));
            else if (categoryName == START_DELAY) {
                menuChoices.setStartDelay(menu.selectedOption(categoryName));
            }
            else if (categoryName == SHOULD_PARK) {
                menuChoices.setShouldPark(menu.selectedOption(categoryName));
            }

        }

        return menuChoices;
    }

    public void makeSomeMoves() throws Exception, InterruptedException {
        MenuChoices menuChoices = getMenuChoices();

        int delay = menuChoices.getStartDelay();
        if (delay > 0)
            Thread.sleep(delay * 1000);

        // Send telemetry message to indicate successful Encoder reset
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         moveTowardBeacon(menuChoices);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

    }

    private void moveTowardBeacon (MenuChoices menuChoices) throws InterruptedException {


        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        if (menuChoices.getAlliance() == ALLIANCE_RED) { //Red alliance
            if (menuChoices.getShouldPark() == SHOULD_PARK_TRUE) {
                if (menuChoices.getStartingPosition() == STARTING_POSITION_RIGHT){
                   robot.leftShooterMotor.setPower(Math.abs(.65));
                    robot.rightShooterMotor.setPower(Math.abs(.65));
                    Thread.sleep(500);
                    robot.intakeMotor.setPower(-.5);
                    Thread.sleep(2 * 1000);
                    robot.intakeMotor.setPower(0);
                    robot.leftShooterMotor.setPower(Math.abs(0));
                    robot.rightShooterMotor.setPower(Math.abs(0));
                    encoderDrive(.6, 90, 90, 5);
                    encoderDrive(.6,-10, -10, 5);
                    encoderDrive(.6, 10, 10, 5);
                }else{
                    robot.leftShooterMotor.setPower(Math.abs(-.55));
                    robot.rightShooterMotor.setPower(Math.abs(-.55));
                    Thread.sleep(500);
                    robot.intakeMotor.setPower(-.5);
                    Thread.sleep(2 * 1000);
                    robot.intakeMotor.setPower(0);
                    robot.leftShooterMotor.setPower(Math.abs(0));
                    robot.rightShooterMotor.setPower(Math.abs(0));
                    encoderDrive(.6, 90, 90, 5);
                    encoderDrive(.6,-10, -10, 5);
                    encoderDrive(.6, 10, 10, 5);
                }
            } else {
            if (menuChoices.getStartingPosition() == STARTING_POSITION_LEFT) { //Red left
                robot.leftShooterMotor.setPower(Math.abs(.55));
                robot.rightShooterMotor.setPower(Math.abs(.55));
                Thread.sleep(500);
                robot.intakeMotor.setPower(-.5);
                Thread.sleep(2 * 1000);
                robot.intakeMotor.setPower(0);
                robot.leftShooterMotor.setPower(Math.abs(0));
                robot.rightShooterMotor.setPower(Math.abs(0));

                encoderDrive(DRIVE_SPEED,12, 12, 5);
                encoderTurn(TURN_SPEED, 41, -41, 4);
                encoderDrive(DRIVE_SPEED, 505, 505, 5);
                encoderTurn(TURN_SPEED, -38, 38, 4);
                encoderDrive(DRIVE_SPEED,15,15,4);

                while (colorSensor1.red() < 2) {

                    telemetry.addData("3 Red1  ", colorSensor1.red());
                    telemetry.addData("5 Blue1 ", colorSensor1.blue());

                    telemetry.update();

                    if (colorSensor1.blue() > colorSensor1.red()) {
                        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        idle();
                        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        sleep(500);
                        encoderDrive(DRIVE_SPEED, 9, 9, 5);
                    }
                }
                robot.colorMotor.setPower(-.1);
                sleep(2000);
                robot.colorMotor.setPower(.1);
                sleep(900);
                robot.colorMotor.setPower(0);

                robot.rightMotor.setPower(.1);
                robot.leftMotor.setPower(0);

                sleep(1100);

                robot.rightMotor.setPower(0);
                robot.leftMotor.setPower(0);

                if (robot.leftMotor.getCurrentPosition() <300){
                    encoderDrive(DRIVE_SPEED, 38,38,5);
                }else {
                    encoderDrive(DRIVE_SPEED, 58,58,5);
                }

                while (colorSensor1.red() < 2) {

                    telemetry.addData("3 Red1  ", colorSensor1.red());
                    telemetry.addData("5 Blue1 ", colorSensor1.blue());

                    telemetry.update();

                    if (colorSensor1.blue() > colorSensor1.red()) {
                        sleep(500);
                        encoderDrive(DRIVE_SPEED, 10, 10, 5);
                    }
                }
                robot.colorMotor.setPower(-.1);
                sleep(2000);
                robot.colorMotor.setPower(.1);
                sleep(900);
                robot.colorMotor.setPower(0);

            } else { //Red right
                robot.leftShooterMotor.setPower(Math.abs(.4));
                robot.rightShooterMotor.setPower(Math.abs(.4));
                Thread.sleep(1000);
                robot.intakeMotor.setPower(-.5);
                Thread.sleep(5 * 1000);
                robot.intakeMotor.setPower(0);
                robot.leftShooterMotor.setPower(Math.abs(0));
                robot.rightShooterMotor.setPower(Math.abs(0));
                //drive
                encoderDrive(DRIVE_SPEED, 8, 8, 5);
                encoderTurn(TURN_SPEED, 32, -32, 4);
                encoderDrive(DRIVE_SPEED, 705, 705, 5);
                encoderDrive(DRIVE_SPEED, 25, 25, 5);
                encoderTurn(TURN_SPEED, -50, 50, 4);
         }
        }
        }else { //Blue alliance
                if (menuChoices.getShouldPark() == SHOULD_PARK_TRUE) {
                    if (menuChoices.getStartingPosition() == STARTING_POSITION_LEFT){
                        robot.leftShooterMotor.setPower(Math.abs(.65));
                        robot.rightShooterMotor.setPower(Math.abs(.65));
                        Thread.sleep(500);
                         robot.intakeMotor.setPower(-.5);
                    Thread.sleep(2 * 1000);
                    robot.intakeMotor.setPower(0);
                        robot.leftShooterMotor.setPower(Math.abs(0));
                        robot.rightShooterMotor.setPower(Math.abs(0));
                        encoderDrive(.6, 90, 90, 5);
                        encoderDrive(.6,-10, -10, 5);
                        encoderDrive(.6, 10, 10, 5);
                    }else{
                        robot.leftShooterMotor.setPower(Math.abs(.55));
                        robot.rightShooterMotor.setPower(Math.abs(.55));
                        Thread.sleep(500);
                        robot.intakeMotor.setPower(-.5);
                        Thread.sleep(2 * 1000);
                        robot.intakeMotor.setPower(0);
                        robot.leftShooterMotor.setPower(Math.abs(0));
                        robot.rightShooterMotor.setPower(Math.abs(0));
                        encoderDrive(.6, 90, 90, 5);
                        encoderDrive(.6,-10, -10, 5);
                        encoderDrive(.6, 10, 10, 5);
                    }
            } else {
                if (menuChoices.getStartingPosition() == STARTING_POSITION_LEFT) { //Blue left
                    robot.leftShooterMotor.setPower(Math.abs(.4));
                    robot.rightShooterMotor.setPower(Math.abs(.4));
                    Thread.sleep(1000);
                    robot.intakeMotor.setPower(-.5);
                    Thread.sleep(5 * 1000);
                    robot.intakeMotor.setPower(0);
                    robot.leftShooterMotor.setPower(Math.abs(0));
                    robot.rightShooterMotor.setPower(Math.abs(0));
                    //drive
                    encoderDrive(DRIVE_SPEED, 8, 8, 5);
                    encoderTurn(TURN_SPEED, -32, 32, 4);
                    encoderDrive(DRIVE_SPEED, 705, 705, 5);
                    encoderDrive(DRIVE_SPEED, 25, 25, 5);

                } else { //Blue right

                    robot.leftShooterMotor.setPower(Math.abs(.45));
                    robot.rightShooterMotor.setPower(Math.abs(.45));
                    Thread.sleep(1000);
                    robot.intakeMotor.setPower(-.5);
                    Thread.sleep(5 * 1000);
                    robot.intakeMotor.setPower(0);
                    robot.leftShooterMotor.setPower(Math.abs(0));
                    robot.rightShooterMotor.setPower(Math.abs(0));

                    encoderDrive(DRIVE_SPEED,8, 8, 5);
                    encoderTurn(TURN_SPEED, -41, 41, 4);
                    encoderDrive(DRIVE_SPEED, 700, 700, 5);
                    encoderTurn(TURN_SPEED, 38, -38, 4);
                    encoderDrive(DRIVE_SPEED,15,15,4);

                    while (colorSensor2.blue() < 2) {

                        telemetry.addData("3 Red1  ", colorSensor1.red());
                        telemetry.addData("5 Blue1 ", colorSensor1.blue());

                        telemetry.addData("6 Red2  ", colorSensor2.red());
                        telemetry.addData("8 Blue2 ", colorSensor2.blue());
                        telemetry.update();

                        if (colorSensor2.red() > colorSensor2.blue()) {
                            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            idle();
                            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            sleep(500);
                            encoderDrive(DRIVE_SPEED, 9, 9, 5);
                        }
                    }
                    robot.colorMotor.setPower(.1);
                    sleep(2000);
                    robot.colorMotor.setPower(-.1);
                    sleep(700);
                    robot.colorMotor.setPower(0);

                    robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(.1);

                    sleep(900);

                    robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(0);

                    if (robot.leftMotor.getCurrentPosition() <300){
                        encoderDrive(DRIVE_SPEED, 38,38,5);
                    }else {
                        encoderDrive(DRIVE_SPEED, 58,58,5);
                    }

                    while (colorSensor1.red() < 2) {

                        telemetry.addData("3 Red1  ", colorSensor1.red());
                        telemetry.addData("5 Blue1 ", colorSensor1.blue());

                        telemetry.update();

                        if (colorSensor1.blue() > colorSensor1.red()) {
                            sleep(500);
                            encoderDrive(DRIVE_SPEED, 10, 10, 5);
                        }
                    }
                    robot.colorMotor.setPower(-.1);
                    sleep(2000);
                    robot.colorMotor.setPower(.1);
                    sleep(2000);
                    robot.colorMotor.setPower(0);
                }
            }
        }


        // robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
       // robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftMotor.getCurrentPosition(),
                                            robot.rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderTurn(double speed,
                            double leftDegrees, double rightDegrees,
                            double timeoutS) throws InterruptedException{
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftDegrees * DEGREES);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightDegrees * DEGREES);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void exponentialIncrease (DcMotor theMotorA, DcMotor theMoterB, double minSpeed,
                                     double maxSpeed, int increments, double rampTime) {
        double delayIncrement = rampTime/increments;
        double speedInrement = (maxSpeed-minSpeed)/increments;
        double  moterSpeed = minSpeed;

        for (int i=0; i< increments; i++) {
            moterSpeed = moterSpeed + speedInrement;
            theMotorA.setPower(Math.abs(moterSpeed));
            theMoterB.setPower(Math.abs(moterSpeed));
            long actualDelay = (long) (delayIncrement * 1000);
            try {
                Thread.sleep(actualDelay);
            } catch (InterruptedException e) {
            }
        }
    }
    public void exponentialDecrease (DcMotor theMotorA, DcMotor theMotorB, double minSpeed,
                                     double maxSpeed, int increments, double rampTime) {
        double delayIncrement = rampTime/increments;
        double speedInrement = (maxSpeed-minSpeed)/increments;
        double  moterSpeed = maxSpeed;

        for (int i=0; i< increments; i++) {
            moterSpeed = moterSpeed - speedInrement;
            theMotorA.setPower(moterSpeed);
            theMotorB.setPower(moterSpeed);
            long actualDelay = (long) (delayIncrement * 1000);
            try {
                Thread.sleep(actualDelay);
            } catch (InterruptedException e) {
            }


        }
    }
/*
    public boolean isColorDetected(int color,int threshold){
        boolean result = false;
        String colorName = COLOR_NAME_BLACK;
        int colorValue = 0;
        if (color == COLOR_RED){
           colorValue = color_sensor.red();
            colorName = COLOR_NAME_RED;
        }
        else if (color == COLOR_BLUE){
            colorValue = color_sensor.blue();
            colorName = COLOR_NAME_BLUE;
        }
        else if (color == COLOR_GREEN){
            colorValue = color_sensor.green();
            colorName = COLOR_NAME_GREEN;
        }

        if ( colorValue >= threshold){
            result = true;
            //telemetry.addData();
        }

        return result;

    }


 public void initColorSensor(){
        color_sensor = hardwareMap.colorSensor.get("color");
     color_sensor.red();     //red
     color_sensor.green();   //green
     color_sensor.blue();    //blue

     color_sensor.alpha();   //total luminosity
     color_sensor.argb();    //Hue measurment
     color_sensor.enableLed(true);
 }
*/
private void shutDownRobot() throws InterruptedException{
    robot.rightMotor.setPower(0);
    robot.leftMotor.setPower(0);
//    robot.bumper.setPosition(MAX_POSITION);
//    robot.rightColorArm.setPosition(COLOR_SERVO_START);
//    robot.leftColorArm.setPosition(COLOR_SERVO_START);
}

}
