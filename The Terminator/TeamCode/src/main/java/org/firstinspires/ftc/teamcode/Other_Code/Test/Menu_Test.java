package org.firstinspires.ftc.teamcode.Other_Code.Test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Other_Code.Extra.Category;
import org.firstinspires.ftc.teamcode.Other_Code.Extra.HardwareSpeedy;
import org.firstinspires.ftc.teamcode.Other_Code.Extra.MenuChoices;
import org.firstinspires.ftc.teamcode.Other_Code.Extra.OptionMenu;
import org.firstinspires.ftc.teamcode.Other_Code.Extra.SingleSelectCategory;

/**
 * Created by Cash America on 2/24/2017.
 */

@Autonomous(name = "Menu_Test", group = "Blue_Right")
@Disabled
public class Menu_Test extends LinearOpMode{
    HardwareSpeedy robot = new HardwareSpeedy();
    private ElapsedTime runtime = new ElapsedTime();
    static final int     COUNTS_PER_INCH         = (65);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.4;
    static final double     P_DRIVE_COEFF           = .01; //0.15;     // Larger is more responsive, but also less stable
    int zAccumulated;  //Total rotation left/right
    int heading;
    int target = 0;

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
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro mrGyro;

    private OptionMenu menu;
    @Override
    public void runOpMode() throws InterruptedException{
        try {
            initializeRobot();

            waitForStart();
            runtime.reset();

            makeSomeMoves();

            shutDownRobot();

        } catch (Exception ex) {
            telemetry.addData("Error", ex.getMessage());
        }
    }
    private void initializeRobot () throws InterruptedException, IllegalStateException, Exception {
        robot.init(hardwareMap);
        InitializeMenu();

        robot.leftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rightMotor.setDirection(DcMotor.Direction.FORWARD);


        colorSensor1 = hardwareMap.colorSensor.get("color 1");
        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x3a)); //Specify I2C Address of color sensor

        colorSensor2 = hardwareMap.colorSensor.get("color 3");
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x3c)); //Specify I2C Address of color sensor

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

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.colorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void makeSomeMoves() throws Exception,InterruptedException {
        MenuChoices menuChoices = getMenuChoices();

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
    public void moveTowardBeacon(MenuChoices menuChoices){
        telemetry.addData("Status", "Running: " + runtime.toString());

        if (menuChoices.getAlliance() == ALLIANCE_RED) {
            drive((40*COUNTS_PER_INCH),.2);
        }else {
                    drive((30*COUNTS_PER_INCH),.2);
        }
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
    private void shutDownRobot() throws InterruptedException{
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
    }
}
