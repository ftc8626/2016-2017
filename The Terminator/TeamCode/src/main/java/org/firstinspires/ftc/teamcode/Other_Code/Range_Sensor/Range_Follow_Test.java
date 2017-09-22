package org.firstinspires.ftc.teamcode.Other_Code.Range_Sensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * Created by Stephen McConnell on 3/13/2017.
 */
@TeleOp(name = "Range_Follow_Test", group = "Wall")
@Disabled
public class Range_Follow_Test extends LinearOpMode{

    //    OpticalDistanceSensor ODS;
    HardwareSpeedremake robot = new HardwareSpeedremake();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    static final int     COUNTS_PER_INCH         = (65);
    static final double     P_DRIVE_COEFF           = .01; //0.15;     // Larger is more responsive, but also less stable
    static final double     DEGREES              = (65 * ((22*Math.PI)/360));

    int zAccumulated;  //Total rotation left/right
    int heading;

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);

        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        // TODO: Why is the following statement needed?
        runtime.reset();

            while (opModeIsActive()) {
                double range = getRange();
                telemetry.addData("UltaSound Range", range);
                telemetry.addData("Motor Left", robot.leftMotor.getPower());
                telemetry.addData("Motor Right", robot.rightMotor.getPower());
                telemetry.update();

                if (range < 20) {
                    robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(0);
//                    robot.range.setPosition(.4);
                    turnAbsolute(-90);

                        double leftPower5 = (range * -.04285714) + 0.55714286;
                        double rightPower5 = (range * .04285714) - 0.25714286;

                        telemetry.addData("leftPower", leftPower5);
                        telemetry.addData("rightPower", rightPower5);

                        leftPower5 = Range.clip(leftPower5, 0, 1);
                        rightPower5 = Range.clip(rightPower5, 0, 1);

                        robot.leftMotor.setPower(leftPower5);
                        robot.rightMotor.setPower(rightPower5);
                } else {
                    robot.leftMotor.setPower(.3);
                    robot.rightMotor.setPower(.3);
                }
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
    public void drive(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double error;
        double steer;

        double target = mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = robot.leftMotor.getCurrentPosition();  //Starting position

        while (robot.leftMotor.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
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

            robot.leftMotor.setPower(leftSpeed);
            robot.rightMotor.setPower(rightSpeed);

            telemetry.addData("Left", robot.leftMotor.getPower());
            telemetry.addData("Right", robot.rightMotor.getPower());
            telemetry.addData("Distance to go", duration + startPosition - robot.leftMotor.getCurrentPosition());
            telemetry.addData("Heading", heading);
            telemetry.update();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target) {
        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 3) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                robot.leftMotor.setPower(turnSpeed);
                robot.rightMotor.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                robot.leftMotor.setPower(-turnSpeed);
                robot.rightMotor.setPower(turnSpeed);
            }

            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.addData("Heading", heading);
            telemetry.update();
        }

        robot.leftMotor.setPower(0);  //Stop the motors
        robot.rightMotor.setPower(0);

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
}

