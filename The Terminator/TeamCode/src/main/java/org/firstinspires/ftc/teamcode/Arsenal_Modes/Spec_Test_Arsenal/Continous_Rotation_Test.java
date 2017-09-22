package org.firstinspires.ftc.teamcode.Arsenal_Modes.Spec_Test_Arsenal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by Stephen McConnell on 3/14/2017.
 */
@Autonomous(name = "Rotot", group = "Blue_Right")
@Disabled
public class Continous_Rotation_Test extends LinearOpMode {
    CRServo rotot = null;
    @Override
    public void runOpMode() {
        rotot = hardwareMap.crservo.get("rotot");
        rotot.setDirection(CRServo.Direction.FORWARD);
        rotot.setPower(0);

        waitForStart();

            rotot.setPower(1);
            sleep(5000);
            rotot.setPower(-1);
            sleep(5000);
            rotot.setPower(0);
            sleep(5000);
        telemetry.addData("Rotot Power", rotot.getPower());
        telemetry.update();
    }
    public void sleep(int milis){
        try {
            Thread.sleep(milis);
        }catch (Exception e){}
    }
}