package org.firstinspires.ftc.teamcode.Other_Code.Optical_Distance_Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Stephen McConnell on 1/8/2017.
 */

@TeleOp(name = "something", group = "something")
@Disabled
public class ODS_Test extends LinearOpMode {
    OpticalDistanceSensor ODS;

    DcMotor mLeft;
    DcMotor mRight;

    double odsReadingRaw;
    static double odsReadingLinear;

    @Override
    public void runOpMode() throws InterruptedException {
        //ODS = hardwareMap.opticalDistanceSensor.get("ods");
        mLeft = hardwareMap.dcMotor.get("ml");
        mRight = hardwareMap.dcMotor.get("mr");

        mLeft.setDirection(DcMotor.Direction.REVERSE);

        mRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            odsReadingRaw = ODS.getRawLightDetected();
            odsReadingLinear = Math.pow(odsReadingRaw, -0.5);

            mRight.setPower(odsReadingLinear * 2);
            mLeft.setPower(.5 - (odsReadingLinear * 2));
            telemetry.addData("1 ODS linear", odsReadingLinear);
        }
    }

}
