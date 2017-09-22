package org.firstinspires.ftc.teamcode.Other_Code.Test;

/**
 * Created by Cash America on 3/9/2017.
 */
/*
public class Boring_Code extends OpMode {
    HardwareSpeedy robot      = new HardwareSpeedy(); // use the class created to define a Pushbot's hardware

    // summary of new algorithm:
// if the value is small (near zero) let it be zero
// if the value is positive, let it start at 0.14 and rise to 1.0 from there
// if the value is negative, let it start at -0.14 and fall to -1.0 from there

// input is gpx (game-pad-x component)
// output is the return value, scaledPower, the altered power setting

   static final double minToMove=0.14;
   static final double scaleTheRestOfTheWay=1.0-minToMove; // 0.86 in this case
        @Override
        public void init() {
        robot.init(hardwareMap);
        }
        @Override
        public void init_loop() {}
        @Override
        public void start() {}
        @Override
        public void loop() {
        double gpx;
        double scaledPower;
        gpx = -gamepad1.left_stick_y;
        scaledPower = 0.0;

        if (Math.abs(gpx) < 0.05) {
            scaledPower = 0.0;
        }else if (gpx > 0.0) {
            scaledPower = minToMove + scaleTheRestOfTheWay * gpx;
        }else if (gpx < 0.0) {
            scaledPower = -minToMove + scaleTheRestOfTheWay * gpx; // no need to subtract, gpx is already negative
        }
        return scaledPower;
        robot.rightMotor.setPower(scaledPower);
        robot.leftMotor.setPower(scaledPower);



    }
}
*/