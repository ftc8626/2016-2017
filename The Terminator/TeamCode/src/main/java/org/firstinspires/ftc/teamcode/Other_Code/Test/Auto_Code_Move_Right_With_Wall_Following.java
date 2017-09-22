package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedremake;

/**
 * Created by Stephen McConnell on 2/23/2017.
 */
@Autonomous(name = "Auto_Code_Move_Red_Right_Wall", group = "Blue_Right")
@Disabled
public class Auto_Code_Move_Right_With_Wall_Following extends LinearOpMode{
    HardwareSpeedremake robot   = new HardwareSpeedremake();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    static final int     COUNTS_PER_INCH         = (65);
    // static final double     P_TURN_COEFF            = .01; //0.1     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = .01; //0.15;     // Larger is more responsive, but also less stable
    static final double     DEGREES              = (65 * ((22*Math.PI)/360));
    static final int     HEADING_SET         = 530;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    DcMotor leftMotor   = null;
    DcMotor rightMotor  = null;
    DcMotor colorMotor = null;
    ColorSensor colorSensor1;  //Instance of ColorSensor - for reading color
    ColorSensor colorSensor2;  //Instance of ColorSensor - for reading color

    int zAccumulated;  //Total rotation left/right
    int heading;
    int target = 0;  //Desired angle to turn to

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    @Override
    public void runOpMode()throws InterruptedException{

        robot.init(hardwareMap);

        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        colorSensor1 = hardwareMap.colorSensor.get("color 1");
        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x3a)); //Specify I2C Address of color sensor

        colorSensor2 = hardwareMap.colorSensor.get("color 3");
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x3c)); //Specify I2C Address of color sensor
        telemetry.addData("Status", "Initialized");

        leftMotor = hardwareMap.dcMotor.get("left_drive");  //Config File
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        colorMotor = hardwareMap.dcMotor.get("color_motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);  //This robot has two gears between motors and wheels. If your robot does not, you will need to reverse only the opposite motor

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

        telemetry.addData("Status", "Calibrating Gyro");
        telemetry.update();
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        while (mrGyro.isCalibrating()) { //Ensure calibration is complete (usually 2 seconds)
            sleep(50);
            idle();
        }
        telemetry.addData("Status", "Robot Ready.");
        telemetry.update();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        colorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        runtime.reset();


        telemetry.addData("Status", "Running: " + runtime.toString());


        //auto drive

        drive((17*COUNTS_PER_INCH), .5);
        turnAbsolute(66);
        //sleep(2000);
        drive((108*COUNTS_PER_INCH), .5);
        turnAbsolute(-2);
        //sleep(2000);
        drive((3 *COUNTS_PER_INCH), .5);
//
//        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        idle();
//        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        while (robot.rightMotor.getCurrentPosition() < (3*COUNTS_PER_INCH)) {
//            double range = getRange();
//            double leftPower = (range * -.07142857)+0.92857143;
//            double rightPower =(range * .07142857)-0.42857143;
//
//            leftPower=Range.clip(leftPower,0,1);
//            rightPower = Range.clip(rightPower,0,1);
//
//            robot.leftMotor.setPower(leftPower);
//            robot.rightMotor.setPower(rightPower);
//        }
//        sleep(1000);

        //sensing beacon
        //sensing beacon
        while (colorSensor2.red() < 2) {

            telemetry.addData("Red  ", colorSensor2.red());
            telemetry.addData("Blue ", colorSensor2.blue());
            telemetry.update();

            if (colorSensor2.blue() >= colorSensor2.red()) {
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                idle();
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(250);
                drive((4*COUNTS_PER_INCH), .3);
            }
        }
        //hitting beacon

        colorMotor.setPower(.1);
        sleep(2000);
        colorMotor.setPower(-.2);
        sleep(1000);
        colorMotor.setPower(0);
       // turnAbsolute(4);

        if (robot.leftMotor.getCurrentPosition() <300){
           // drive((57*COUNTS_PER_INCH), .5);

            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //second button
            while (opModeIsActive() && robot.rightMotor.getCurrentPosition() < (57*COUNTS_PER_INCH)) {
//                if (robot.rightMotor.getCurrentPosition() > (15*COUNTS_PER_INCH)){
//                    telemetry.addData("Status", "Past 1st Beacon");
//                    if (colorSensor2.red() < 3){
//                        telemetry.addData("Status", "Found 2nd Beacon");
//                        break;
//                    }else{
//                        telemetry.addData("Status", "Following Wall");
//                    }
//                }else {
//                    telemetry.addData("Status", "Not Past 1st Beacon");
//                }
                    double range = getRange();
                    double leftPower = (range * -.07142857)+0.92857143;
                    double rightPower =(range * .07142857)-0.42857143;
                    leftPower=Range.clip(leftPower,0,1);
                    rightPower = Range.clip(rightPower,0,1);

                    robot.leftMotor.setPower(leftPower);
                    robot.rightMotor.setPower(rightPower);
            }
        }else {
            //drive((63*COUNTS_PER_INCH),.5);

            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && robot.rightMotor.getCurrentPosition() < (62*COUNTS_PER_INCH)) {
//                if (robot.rightMotor.getCurrentPosition() > (15*COUNTS_PER_INCH)){
//                    telemetry.addData("Status", "Past 1st Beacon");
//                    telemetry.update();
//                    if (colorSensor2.red() < 3){
//                        telemetry.addData("Status", "Found 2nd Beacon");
//                        telemetry.update();
//                        break;
//                    }else{
//                        telemetry.addData("Status", "Following Wall");
//                        telemetry.update();
//                    }
//                }else {
//                    telemetry.addData("Status", "Not Past 1st Beacon");
//                    telemetry.update();
//                }
                double range = getRange();
                double leftPower = (range * -.07142857)+0.92857143;
                double rightPower =(range * .07142857)-0.42857143;

                leftPower=Range.clip(leftPower,0,1);
                rightPower = Range.clip(rightPower,0,1);

                robot.leftMotor.setPower(leftPower);
                robot.rightMotor.setPower(rightPower);
            }
        }
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);

        while (colorSensor2.red() < 2) {

            telemetry.addData("3 Red1  ", colorSensor2.red());
            telemetry.addData("5 Blue1 ", colorSensor2.blue());
            telemetry.update();

            if (colorSensor2.blue() > colorSensor2.red()){
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                idle();
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(250);
                drive((4*COUNTS_PER_INCH), .3);
            }
        }
        colorMotor.setPower(.1);
        sleep(2000);
        colorMotor.setPower(-.2);
        sleep(800);
        colorMotor.setPower(0);
        turnAbsolute(42);

//        robot.leftShooterMotor.setPower(Math.abs(.65));
//        robot.rightShooterMotor.setPower(Math.abs(.65));
//        sleep(800);
//        robot.intakeMotor.setPower(-.5);
//        robot.bumper.setPosition(BUMPER_UP);
//        sleep(2 * 1000);
//        robot.intakeMotor.setPower(0);
//        robot.bumper.setPosition(BUMPER_DOWN);
//        robot.leftShooterMotor.setPower(Math.abs(0));
//        robot.rightShooterMotor.setPower(Math.abs(0));
//
//        colorMotor.setPower(.2);
//        sleep(500);
//        colorMotor.setPower(0);
//        drive(45, -1);


        //drive((-10*COUNTS_PER_INCH),-1);
    }



    public void drive(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double error;
        double steer;

        double target = mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = leftMotor.getCurrentPosition();  //Starting position

        while (leftMotor.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            zAccumulated = mrGyro.getIntegratedZValue();  //Current direction
            heading = mrGyro.getHeading();

//            leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
//            rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation

            error = getError(target);
            steer = getSteer(error, P_DRIVE_COEFF);

            leftSpeed = power - steer;
            rightSpeed = power + steer;

            //added in ^ old stuff \|/
            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            leftMotor.setPower(leftSpeed);
            rightMotor.setPower(rightSpeed);

            telemetry.addData("Left", leftMotor.getPower());
            telemetry.addData("Right", rightMotor.getPower());
            telemetry.addData("Distance to go", duration + startPosition - leftMotor.getCurrentPosition());
            telemetry.addData("Heading", heading);
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    //This function turns a number of degrees compared to where the robot is. Positive numbers trn left.
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + mrGyro.getIntegratedZValue());
    }

    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target) {
        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 3) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                leftMotor.setPower(turnSpeed);
                rightMotor.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                leftMotor.setPower(-turnSpeed);
                rightMotor.setPower(turnSpeed);
            }

            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.addData("Heading", heading);
            telemetry.update();
        }

        leftMotor.setPower(0);  //Stop the motors
        rightMotor.setPower(0);

    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - mrGyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftDegrees * DEGREES);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightDegrees * DEGREES);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    private double getRange() {
        double ultraSonicRange = 0;

        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        byte byteUltraSonicRange = range1Cache[0];

        // The following line uses implicit type casting don't by the JVM
        // bytes -> short -> int -> long -> double, etc...
        ultraSonicRange = byteUltraSonicRange;



        telemetry.addData("Ultra Sonic (byte)", byteUltraSonicRange & 0xFF);
        telemetry.addData("Ultra Sonic (double)", ultraSonicRange);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        return ultraSonicRange;
    }
}
