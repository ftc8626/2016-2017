package org.firstinspires.ftc.teamcode.Arsenal_Modes.Autonomous.Red;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen McConnell on 2/13/2017.
 */
@Autonomous(name = "Arsenal Auto Red Right", group = "Red")
//@Disabled
public class Red_Right extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    static final int     COUNTS_PER_INCH         = (65);
   // static final double     P_TURN_COEFF            = .01; //0.1     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = .01; //0.15;     // Larger is more responsive, but also less stable
    static final double     DEGREES              = (65 * ((22*Math.PI)/360));
    static final int     HEADING_SET         = 530;

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    byte[] range2Cache;

    public I2cDevice RANGE1;
    public I2cDevice RANGE2;
    public I2cDeviceSynch RANGE1Reader;
    public I2cDeviceSynch RANGE2Reader;
    DcMotor leftFrontMotor   = null;
    DcMotor rightFrontMotor  = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    ColorSensor colorSensor1;  //Instance of ColorSensor - for reading color
    ColorSensor colorSensor2;  //Instance of ColorSensor - for reading color
    Servo caparm;
    CRServo rotot = null;

    int zAccumulated;  //Total rotation left/right
    int heading;
    int target = 0;  //Desired angle to turn to

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    @Override
    public void runOpMode()throws InterruptedException{

        colorSensor1 = hardwareMap.colorSensor.get("color 1");
        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x3a)); //Specify I2C Address of color sensor

        colorSensor2 = hardwareMap.colorSensor.get("color 3");
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x3c)); //Specify I2C Address of color sensor
        telemetry.addData("Status", "Initialized");

        RANGE1 = hardwareMap.i2cDevice.get("range1");
        RANGE2 = hardwareMap.i2cDevice.get("range2");

        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, I2cAddr.create8bit(0x28), false);
        RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, I2cAddr.create8bit(0x30), false);

        RANGE1Reader.engage();
        RANGE2Reader.engage();

        leftBackMotor = hardwareMap.dcMotor.get("left_back_drive");
        leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        rightBackMotor = hardwareMap.dcMotor.get("right_back_drive");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        rotot = hardwareMap.crservo.get("color_pusher");
        caparm = hardwareMap.servo.get("caparm");
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        runtime.reset();


        telemetry.addData("Status", "Running: " + runtime.toString());


        //auto drive
        caparm.setPosition(.77);

        drive((15*COUNTS_PER_INCH), .5);
        turnAbsolute(-60);
        //sleep(2000);
        drive((64*COUNTS_PER_INCH), .5);
        halfTurn(-10);
        //sleep(2000);
        //drive((3*COUNTS_PER_INCH), .5);

        while (opModeIsActive() && rightFrontMotor.getCurrentPosition() <= (60*COUNTS_PER_INCH)) {
            double range = getRange1();

            double leftPower5 = (range * -.07142857) + 0.92857143;
            double rightPower5 = (range * .07142857) - 0.42857143;

            telemetry.addData("leftPower", leftPower5);
            telemetry.addData("rightPower", rightPower5);

            leftPower5 = Range.clip(leftPower5, 0, 1);
            rightPower5 = Range.clip(rightPower5, 0, 1);

            leftBackMotor.setPower(leftPower5);
            leftFrontMotor.setPower(leftPower5);
            rightBackMotor.setPower(rightPower5);
            rightFrontMotor.setPower(rightPower5);

            telemetry.addData("UltaSound Range", range);
            telemetry.addData("Motor Left", leftBackMotor.getPower());
            telemetry.addData("Motor Right", rightFrontMotor.getPower());

            telemetry.update();
        }
        rightFrontMotor.setPower(-.3);
        leftFrontMotor.setPower(-.3);
        rightBackMotor.setPower(-.3);
        leftBackMotor.setPower(-.3);


        //sensing beacon
        while (colorSensor2.blue() < 3) {

            telemetry.addData("3 Red1  ", colorSensor2.red());
            telemetry.addData("5 Blue1 ", colorSensor2.blue());
            telemetry.update();

            if (colorSensor2.red() >= colorSensor2.blue()) {
                rightFrontMotor.setPower(-.3);
                leftFrontMotor.setPower(-.3);
                rightBackMotor.setPower(-.3);
                leftBackMotor.setPower(-.3);
            }
        }
        //hitting beacon
        encoderDrive(.3,5,5,5);
        rotot.setPower(-.1);
        sleep(2000);
        rotot.setPower(.2);
        sleep(1000);
        rotot.setPower(0);

        rightFrontMotor.setPower(-.3);
        leftFrontMotor.setPower(-.3);
        rightBackMotor.setPower(-.3);
        leftBackMotor.setPower(-.3);

        while (colorSensor2.blue() < 3) {

            telemetry.addData("3 Red1  ", colorSensor2.red());
            telemetry.addData("5 Blue1 ", colorSensor2.blue());
            telemetry.update();

            if (colorSensor2.red() >= colorSensor2.blue()){
//                rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                idle();
//                rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                sleep(250);
//                drive((4*COUNTS_PER_INCH), .3);
                rightFrontMotor.setPower(-.3);
                leftFrontMotor.setPower(-.3);
                rightBackMotor.setPower(-.3);
                leftBackMotor.setPower(-.3);
            }
        }
        encoderDrive(.3,5,5,5);
        rotot.setPower(-.1);
        sleep(2000);
        rotot.setPower(.2);
        sleep(800);
        rotot.setPower(0);

        //drive((-10*COUNTS_PER_INCH),-1);
    }



    public void drive(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double error;
        double steer;

        double target = mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = leftBackMotor.getCurrentPosition();  //Starting position

        while (leftBackMotor.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
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

            leftBackMotor.setPower(leftSpeed);
            rightBackMotor.setPower(rightSpeed);
            leftFrontMotor.setPower(leftSpeed);
            rightFrontMotor.setPower(rightSpeed);

            telemetry.addData("Left", leftBackMotor.getPower());
            telemetry.addData("Right", rightBackMotor.getPower());
            telemetry.addData("Distance to go", duration + startPosition - leftBackMotor.getCurrentPosition());
            telemetry.addData("Heading", heading);
            telemetry.update();
        }

        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    //This function turns a number of degrees compared to where the robot is. Positive numbers trn left.
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + mrGyro.getIntegratedZValue());
    }

    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target) {
        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.3;

        while (Math.abs(zAccumulated - target) > 3) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                leftBackMotor.setPower(turnSpeed);
                leftFrontMotor.setPower(turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                rightBackMotor.setPower(turnSpeed);
                rightFrontMotor.setPower(turnSpeed);
            }

            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.addData("Heading", heading);
            telemetry.update();
        }

        leftBackMotor.setPower(0);  //Stop the motors
        rightBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

    }
    public void halfTurn(int target) {
        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.3;

        while (Math.abs(zAccumulated - target) > 3) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontMotor.setPower(turnSpeed);
                rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontMotor.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftBackMotor.setPower(-turnSpeed);
                rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightBackMotor.setPower(turnSpeed);
            }

            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.addData("Heading", heading);
            telemetry.update();
        }

        leftBackMotor.setPower(0);  //Stop the motors
        rightBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

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
            newLeftTarget = leftBackMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightBackMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftBackMotor.setTargetPosition(newLeftTarget);
            leftFrontMotor.setTargetPosition(newLeftTarget);
            rightBackMotor.setTargetPosition(newRightTarget);
            rightFrontMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftBackMotor.setPower(Math.abs(speed));
            leftFrontMotor.setPower(Math.abs(speed));
            rightBackMotor.setPower(Math.abs(speed));
            rightFrontMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftBackMotor.isBusy() && rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftBackMotor.getCurrentPosition(),
                        rightBackMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftBackMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            rightFrontMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            newLeftTarget = leftBackMotor.getCurrentPosition() + (int)(leftDegrees * DEGREES);
            newRightTarget = rightBackMotor.getCurrentPosition() + (int)(rightDegrees * DEGREES);
            leftBackMotor.setTargetPosition(newLeftTarget);
            leftFrontMotor.setTargetPosition(newLeftTarget);
            rightBackMotor.setTargetPosition(newRightTarget);
            rightFrontMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftBackMotor.setPower(Math.abs(speed));
            leftFrontMotor.setPower(Math.abs(speed));
            rightBackMotor.setPower(Math.abs(speed));
            rightFrontMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftBackMotor.isBusy() && rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftBackMotor.getCurrentPosition(),
                        rightBackMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftBackMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            rightFrontMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }
    private double getRange1() {
        double ultraSonicRange1 = 0;

        range1Cache = RANGE1Reader.read(0x04,2);

        byte byteUltraSonicRange1 = range1Cache[0];

        // The following line uses implicit type casting don't by the JVM
        // bytes -> short -> int -> long -> double, etc...
        ultraSonicRange1 = byteUltraSonicRange1;



//        telemetry.addData("Ultra Sonic (byte)", byteUltraSonicRange & 0xFF);
//        telemetry.addData("Ultra Sonic (double)", ultraSonicRange);
//
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.update();

        return ultraSonicRange1;
    }

    private double getRange2() {
        double ultraSonicRange2 = 0;

        range2Cache = RANGE2Reader.read(0x04,2);

        byte byteUltraSonicRange2 = range2Cache[0];

        // The following line uses implicit type casting don't by the JVM
        // bytes -> short -> int -> long -> double, etc...
        ultraSonicRange2 = byteUltraSonicRange2;



//        telemetry.addData("Ultra Sonic (byte)", byteUltraSonicRange & 0xFF);
//        telemetry.addData("Ultra Sonic (double)", ultraSonicRange);
//
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.update();

        return ultraSonicRange2;
    }
}
