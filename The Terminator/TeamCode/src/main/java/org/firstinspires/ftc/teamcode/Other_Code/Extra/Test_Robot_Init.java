package org.firstinspires.ftc.teamcode.Other_Code.Extra;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen McConnell on 2/28/2017.
 */

public class Test_Robot_Init {
    /* Public OpMode members. */
    public DcMotor leftBackMotor   = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor rightBackMotor  = null;
    public DcMotor rightFrontMotor  = null;
    public DcMotor intakeMotor = null;
    public DcMotor rightShooterMotor = null;
    public DcMotor leftShooterMotor = null;
    public DcMotor liftMotor = null;
    public Servo   shooterFlicker = null;
    public Servo   flick = null;
//    public Servo rangeR = null;
//    public Servo rangeL = null;
    public CRServo rotot = null;
    public Servo   capArm = null;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Test_Robot_Init(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftBackMotor = hwMap.dcMotor.get("left_back_drive");
        leftFrontMotor = hwMap.dcMotor.get("left_front_drive");
        rightBackMotor = hwMap.dcMotor.get("right_back_drive");
        rightFrontMotor = hwMap.dcMotor.get("right_front_drive");
        rightShooterMotor = hwMap.dcMotor.get("right_shooter");
        leftShooterMotor = hwMap.dcMotor.get("left_shooter");
        liftMotor = hwMap.dcMotor.get("lift");
        intakeMotor = hwMap.dcMotor.get("intakeMotor");
        shooterFlicker = hwMap.servo.get("shooter_flicker");
        flick = hwMap.servo.get("flick");
        capArm = hwMap.servo.get("caparm");
//        rangeR = hwMap.servo.get("range1");
//        rangeL = hwMap.servo.get("range2");
        rotot = hwMap.crservo.get("color_pusher");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        rightShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        leftShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotot.setDirection(CRServo.Direction.FORWARD);



        // Set all motors to zero power
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        intakeMotor.setPower(0);
        rightShooterMotor.setPower(0);
        leftShooterMotor.setPower(0);
        liftMotor.setPower(0);
        rotot.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterFlicker.setPosition(.35);
        flick.setPosition(.5);
        capArm.setPosition(.75);
//        rangeL.setPosition(0);
//        rangeR.setPosition(1);
    }

    /***
     *
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
