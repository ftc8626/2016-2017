package org.firstinspires.ftc.teamcode.How_To_Code_Again;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Cash America on 8/12/2017.
 */

public class Hardwear {
    /* public OpMode. */
    public DcMotor leftMotor    = null;
    public DcMotor rightMotor   = null;
    public Servo    servo1      = null;

    /* local OpMode. */
    HardwareMap hwMap       =null;
    private ElapsedTime period  = new ElapsedTime();

    /* Initialize standard Hardwearr interface */
    public void init (HardwareMap ahwMap) {
        //Save reference to Hardware map
        hwMap = ahwMap;

        //Define and Initialize Motors
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");
        servo1 = hwMap.servo.get("servo 1");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo1.setPosition(0);
    }
}