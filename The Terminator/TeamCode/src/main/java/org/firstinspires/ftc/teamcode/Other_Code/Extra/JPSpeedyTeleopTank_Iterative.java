package org.firstinspires.ftc.teamcode.Other_Code.Extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;
/*
     Note: removed comment related to license!
 */

@TeleOp(name="JPSpeedyTest: Teleop Tank", group="Speedy")
@Disabled
public class JPSpeedyTeleopTank_Iterative extends OpMode {


    public static int SCALING_TYPE_LINEAR = 0;
    public static int SCALING_TYPE_EXPONENTIAL = 1;
    private boolean isTankControls = false;
    private int scalingType = SCALING_TYPE_LINEAR;

    /* Declare OpMode members. */
    HardwareSpeedy robot      = new HardwareSpeedy(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    // servoGate positions
    double          clawOffset  = .5;                  // Servo mid position

    //final double    CLAW_SPEED  = 1 ;                 // sets rate to move servo

    private final double SHOOTER_OUT_SPEED = .2;
    private final double SHOOTER_IN_SPEED = -.1;
    double SHOOTER_MOTOR_POWER_FACTOR = .8;

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
        /*
        robot.shooterMotor.setPower(1);
        robot.intakeMotor.setPower(1);
        */
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.guide && gamepad2.guide) {
            // In case of an emergency!
            stopAllMotors();
            moveServosToStartingPositions();
        }

//        if (gamepad1.a) {
//            // Use exponential scaling of joystick inputs.
//            setScalingType(SCALING_TYPE_EXPONENTIAL);
//        } else {
            // Use linear scaling of joystick inputs.
            setScalingType(SCALING_TYPE_LINEAR);
//        }

//        if (gamepad1.b) {
//            // Use single joystick (i.e. "steering style") driving controls.
//            setIsTankControls(false);
//        } else {
            // Use two joystick (a.k.a. "tank style") driving controls.
            setIsTankControls(true);
//        }

        if(isTankControls) {
            // Use "tank style" driving controls.
            driveWithTankControls();
        } else {
            // Use one stick "steering style" driving controls
            driveWithSteeringControls();
        }

        // shooter speed

        // Send telemetry message to signify robot running;            // Send telemetry message to signify robot running;
//        telemetry.addData("left", "%.2f", left);
//        telemetry.addData("right", "%.2f", right);
//        updateTelemetry(telemetry);
    }

    public void driveWithTankControls() {

        // Note: The joystick goes negative when pushed forwards, so negate it!
        robot.leftMotor.setPower(scaleInput(getScalingType(), - gamepad1.left_stick_y));
        robot.rightMotor.setPower(scaleInput(getScalingType(), - gamepad1.right_stick_y));
    }


    public void driveWithSteeringControls() {

        // Throttle uses left joy stick Y axis.
        double throttle = scaleInput(getScalingType(), gamepad1.left_stick_y);
        // Turn uses left joy stick X axis.
        double turn     = scaleInput(getScalingType(), gamepad1.left_stick_x);

        // TODO: Find out why left speed has compile error!!!
//        double leftspeed  = throttle – turn;
        double rightspeed = throttle + turn;
//        robot.leftMotor.setPower(leftspeed);
        robot.rightMotor.setPower(rightspeed);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void setScalingType(int scalingType) {
        this.scalingType = scalingType;
    }

    public int getScalingType() {
        return this.scalingType;
    }

    public void setIsTankControls(boolean trueOrFalse) {
        this.isTankControls = trueOrFalse;
    }

    public boolean isTankControls() {
        return this.isTankControls;
    }

    public void stopAllMotors() {

        robot.intakeMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.rightShooterMotor.setPower(0);
        robot.leftShooterMotor.setPower(0);
    }

    public void moveServosToStartingPositions() {
//
//        robot.rightColorArm.setPosition(COLOR_SERVO_START);
//        robot.leftColorArm.setPosition(COLOR_SERVO_START);
        robot.bumper.setPosition(MAX_POSITION);
    }

    public double scaleInput(int scalingType, float inputValue) {
        double outputValue;

        if (scalingType == SCALING_TYPE_EXPONENTIAL) {
            outputValue = exponentialyScaleInput(inputValue);
        } else {
            outputValue = linearlyScaleInput(inputValue);
        }

        return outputValue;
    }

    public double exponentialyScaleInput(float inputValue) {
        double outputValue;

        // TODO: Document what the different hard-coded constants are used to tune!

        if (Math.abs(inputValue) > 5) {
            float step1 = inputValue * inputValue;
            float step2 = step1 * inputValue;
            float step3 = step2 * 80;
            float step4 = step3 / 16129;
            double step5 = step4 / Math.abs(inputValue);
            double step6 = 20 * inputValue;
            double step7 = step6 / Math.abs(inputValue);
            double rightExp = step5 + step7;
            outputValue = rightExp;
        } else {
            outputValue = 0.0E00;
        }

        return outputValue;
    }

    public double linearlyScaleInput(float inputValue) {
        double outputValue;
//        float a = 0; // a = 0 : linear, a > 0 : exponential, etc... family of scaling
        float b = 0; // "inverse deadband" for linear scaling

        // TODO: Document what the different hard-coded constants
        // are used to tune!

        if (inputValue >= 0) {
            // g(x) = b + (1-b)*[a*x^3 + (1-a)*x];
            outputValue = b + (1-b)*inputValue;
        } else {
            // g(x) = -b + (1-b)*[a*x^3 + (1-a)*x];
            outputValue = -b + (1-b)*inputValue;
        }

        return outputValue;
    }
}
