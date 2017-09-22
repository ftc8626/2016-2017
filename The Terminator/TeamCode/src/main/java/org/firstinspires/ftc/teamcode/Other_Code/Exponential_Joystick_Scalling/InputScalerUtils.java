package org.firstinspires.ftc.teamcode.Other_Code.Exponential_Joystick_Scalling;

/**
 * Created by John on 12/15/2016.
 */

public class InputScalerUtils {

    public static final int SCALING_TYPE_LINEAR = 0;
    public static final int SCALING_TYPE_EXPONENTIAL = 1;

    private InputScalerUtils() {
    }

    public static double scaleInput(int scalingType, float inputValue) {
        double outputValue;

        if (scalingType == SCALING_TYPE_EXPONENTIAL) {
            outputValue = exponentialyScaleInput(inputValue);
        } else {
            outputValue = linearlyScaleInput(inputValue);
        }

        return outputValue;
    }

    public static double exponentialyScaleInput(float inputValue) {

        double outputValue;

        // TODO: Document what the different hard-coded constants are used to tune!

        float minInput = -1;
        float maxInput = +1;
        float minOutput = -1;
        float maxOutput = +1;
        double deadband = 0.1;
        double a = maxOutput - deadband;
        double b = maxInput;
        double c = deadband;

        if (inputValue > maxInput) {
            inputValue = maxInput;
        } else if (inputValue < minInput) {
            inputValue = minInput;
        }

        if (Math.abs(inputValue) > 0) {
//            float step1 = inputValue * inputValue;
//            float step2 = step1 * inputValue;
//            float step3 = step2 * 80;
//            float step4 = step3 / 16129;
//            double step5 = step4 / Math.abs(inputValue);
//            double step6 = 20 * inputValue;
//            double step7 = step6 / Math.abs(inputValue);
//            double rightExp = step5 + step7;
//            outputValue = rightExp;

            double step1 = inputValue * inputValue;
            double step2 = step1 * inputValue;
            double step3 = step2 * a;
            double step4 = step3 / b;
            double step5 = step4 / Math.abs(inputValue);

            double step6 = c * inputValue;
            double step7 = step6 / Math.abs(inputValue);
            double rightExp = step5 + step7;
            outputValue = rightExp;

        } else {
            outputValue = 0.0E00;
        }

        return outputValue;
    }

    public static double linearlyScaleInput(float inputValue) {

        double outputValue;

//        float a = 0.0; // a = 0 : linear, a 0 : exponential, etc... family of scaling

        double b = 0.1; // "inverse deadband" for linear scaling

        // TODO: Document what the different hard-coded constants are used to tune!

        float minInput = -1;
        float maxInput = +1;
        float minOutput = -1;
        float maxOutput = +1;

        if (inputValue > maxInput) {
            inputValue = maxInput;
        } else if (inputValue < minInput) {
            inputValue = minInput;
        }

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
