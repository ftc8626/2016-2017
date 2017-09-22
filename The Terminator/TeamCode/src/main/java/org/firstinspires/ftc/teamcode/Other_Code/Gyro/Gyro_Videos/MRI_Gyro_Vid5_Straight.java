package org.firstinspires.ftc.teamcode.Other_Code.Gyro.Gyro_Videos;

/*
Modern Robotics Gyro Straight Example
Updated 11/3/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
Gyro Sensor named "gyro"
Motor named "ml" for left drive train
Motor named "mr" for right drive train

For more information, go to http://modernroboticsedu.com/course/view.php?id=4
Support is available by emailing support@modernroboticsinc.com.
*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.Test_Robot_Init;

@TeleOp(name = "5 Straight", group = "Gyro Video")
@Disabled
public class MRI_Gyro_Vid5_Straight extends LinearOpMode {  //Linear op mode is being used so the program does not get stuck in loop()
    Test_Robot_Init robot = new Test_Robot_Init();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        waitForStart();
        runtime.reset();

        while (mrGyro.isCalibrating()) { //Ensure calibration is complete (usually 2 seconds)
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running: " + runtime.toString());

            driveStraight(30000, 0.30); //Drive straight for 30,000 encoder ticks (basically forever)
        }
    }


    public void driveStraight(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;

        double target = mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = robot.leftBackMotor.getCurrentPosition();  //Starting position

        while (robot.leftBackMotor.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            zAccumulated = mrGyro.getIntegratedZValue();  //Current direction

            leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.leftBackMotor.setPower(leftSpeed);
            robot.leftFrontMotor.setPower(leftSpeed);
            robot.rightBackMotor.setPower(rightSpeed);
            robot.rightFrontMotor.setPower(rightSpeed);

            telemetry.addData("Left Back", robot.leftBackMotor.getPower());
            telemetry.addData("Left Front", robot.leftFrontMotor.getPower());
            telemetry.addData("Right Back", robot.rightBackMotor.getPower());
            telemetry.addData("Right Front", robot.rightFrontMotor.getPower());
            telemetry.addData("Distance to go", duration + startPosition - robot.leftBackMotor.getCurrentPosition());
            telemetry.update();
        }

        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
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
                robot.leftBackMotor.setPower(turnSpeed);
                robot.leftFrontMotor.setPower(turnSpeed);
                robot.rightBackMotor.setPower(-turnSpeed);
                robot.rightFrontMotor.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                robot.leftBackMotor.setPower(-turnSpeed);
                robot.leftFrontMotor.setPower(-turnSpeed);
                robot.rightBackMotor.setPower(turnSpeed);
                robot.rightFrontMotor.setPower(turnSpeed);
            }

            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }

        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);

    }

}
