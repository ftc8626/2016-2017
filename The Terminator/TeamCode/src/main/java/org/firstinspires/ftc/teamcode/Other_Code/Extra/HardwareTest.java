package org.firstinspires.ftc.teamcode.Other_Code.Extra;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
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
public class HardwareTest
{
    /* Public OpMode members. */
   public DeviceInterfaceModule CDI;
   public ColorSensor color_Sensor_A;
   public DcMotor leftMotor   = null;
   public DcMotor rightMotor  = null;
   public ColorSensor colorSensorR;
   public ColorSensor colorSensorB;
   public Servo rightColorArm    = null;
   public Servo leftColorArm    = null;
   public Servo bumper    = null;



    public boolean LEDAState = true;

    public static final double BUTTON_PUSHER_RETRACTED = .9;
    public static final double BUTTON_PUSHER_EXTENDED  = .1;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTest(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.dcMotor.get("right_drive");
        rightMotor = hwMap.dcMotor.get("left_drive");
        leftColorArm = hwMap.servo.get("left color");
        rightColorArm = hwMap.servo.get("right_color");
        bumper = hwMap.servo.get("bumper");




        color_Sensor_A = hwMap.colorSensor.get("color 1");
        colorSensorR = hwMap.colorSensor.get("color 2");
        colorSensorB = hwMap.colorSensor.get("color 3");
        color_Sensor_A.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorR.setI2cAddress(I2cAddr.create8bit(0x3a));
        colorSensorB.setI2cAddress(I2cAddr.create8bit(0x3e));
        CDI = hwMap.deviceInterfaceModule.get("Device Interface Module 1");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bumper.setPosition(.5);
        leftColorArm.setPosition(.9);
        rightColorArm.setPosition(.9);

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

