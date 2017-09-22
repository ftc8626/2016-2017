package org.firstinspires.ftc.teamcode.Other_Code.Gyro.Gyro_Videos;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen McConnell on 2/13/2017.
 */
@TeleOp(name = "Auto_Code_Straight", group = "Gyro")
@Disabled
public class Auto_Code_Straight extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;

    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        leftBackMotor = hardwareMap.dcMotor.get("left_back_drive");  //Config File
        rightBackMotor = hardwareMap.dcMotor.get("right_back_drive");
        leftFrontMotor = hardwareMap.dcMotor.get("left_back_drive");  //Config File
        rightFrontMotor = hardwareMap.dcMotor.get("right_back_drive");
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);  //This robot has two gears between motors and wheels. If your robot does not, you will need to reverse only the opposite motor
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        waitForStart();
        runtime.reset();

        while (mrGyro.isCalibrating()) { //Ensure calibration is complete (usually 2 seconds)
        }
            telemetry.addData("Status", "Running: " + runtime.toString());

            driveStraight(13000, 1); //Drive straight for 30,000 encoder ticks (basically forever)

    }


    public void driveStraight(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;

        double target = mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = leftBackMotor.getCurrentPosition();  //Starting position

        while (leftBackMotor.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            zAccumulated = mrGyro.getIntegratedZValue();  //Current direction

            leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            leftBackMotor.setPower(leftSpeed);
            rightBackMotor.setPower(rightSpeed);
            leftFrontMotor.setPower(leftSpeed);
            rightFrontMotor.setPower(rightSpeed);

            telemetry.addData("1. Left Back", leftBackMotor.getPower());
            telemetry.addData("2. Right Back", rightBackMotor.getPower());
            telemetry.addData("1. Left Front", leftFrontMotor.getPower());
            telemetry.addData("2. Right Front", rightFrontMotor.getPower());
            telemetry.addData("3. Distance to go", duration + startPosition - leftBackMotor.getCurrentPosition());
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
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 3) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                leftBackMotor.setPower(turnSpeed);
                rightBackMotor.setPower(-turnSpeed);
                leftFrontMotor.setPower(turnSpeed);
                rightFrontMotor.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                leftBackMotor.setPower(-turnSpeed);
                rightBackMotor.setPower(turnSpeed);
                leftFrontMotor.setPower(-turnSpeed);
                rightFrontMotor.setPower(turnSpeed);
            }

            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }

        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);

    }
}
