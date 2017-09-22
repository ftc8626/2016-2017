package org.firstinspires.ftc.teamcode.Other_Code.Extra;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_dr public DcMotor shooterMotor = null;
 public DcMotor intakeMotor = null;ive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareSpeedremake
{
    /* Public OpMode members. */
   public DeviceInterfaceModule CDI;
   public DcMotor leftMotor   = null;
   public DcMotor rightMotor  = null;
//   public DcMotor rightShooterMotor = null;
//   public DcMotor leftShooterMotor = null;
//   public DcMotor intakeMotor = null;
//   public DcMotor colorMotor = null;
   public Servo bumper    = null;
   public Servo ballRight = null;
   public Servo ballLeft = null;
//   public Servo range = null;


    public static final double BUTTON_PUSHER_RETRACTED = .9;
    public static final double BUTTON_PUSHER_EXTENDED  = .1;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSpeedremake(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");
//        rightShooterMotor = hwMap.dcMotor.get("right_shooter");
//        leftShooterMotor = hwMap.dcMotor.get("left_shooter");
//        intakeMotor = hwMap.dcMotor.get("intakeMotor");
//        colorMotor = hwMap.dcMotor.get("color_motor");
        bumper = hwMap.servo.get("bumper");
        ballLeft = hwMap.servo.get("bl");
        ballRight = hwMap.servo.get("br");
//        range = hwMap.servo.get("range");

        CDI = hwMap.deviceInterfaceModule.get("Device Interface Module 1");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//        rightShooterMotor.setDirection(DcMotor.Direction.FORWARD);
//        leftShooterMotor.setDirection(DcMotor.Direction.REVERSE);
//        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
//        colorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
//        rightShooterMotor.setPower(0);
//        leftShooterMotor.setPower(0);
//        intakeMotor.setPower(0);
//        colorMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        colorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bumper.setPosition(.5);
        ballLeft.setPosition(.9);
        ballRight.setPosition(.1);
//        range.setPosition(1);
    }

    /***
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

