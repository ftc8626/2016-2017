package org.firstinspires.ftc.teamcode.Other_Code.Exponential_Joystick_Scalling;

import junit.framework.TestCase;

/**
 * Created by John on 12/15/2016.
 */
public class InputScalerUtilsTest extends TestCase {

    public static final int SCALING_TYPE_LINEAR = 0;
    public static final int SCALING_TYPE_EXPONENTIAL = 1;

    public void setUp() throws Exception {
        super.setUp();
    }

    public void tearDown() throws Exception {

    }

    public void testScaleInput() throws Exception {

        int xSteps = 20;
        float xMin = -1;
        float xMax = 1;
        float xRange = xMax - xMin;
        float xDelta = xRange / xSteps;

        float xInput = xMin;
        for (int i = 0; i < xSteps; i++) {

            double xLinearOutput = InputScalerUtils.scaleInput(SCALING_TYPE_LINEAR, xInput);
            double xExponentialOutput = InputScalerUtils.scaleInput(SCALING_TYPE_EXPONENTIAL, xInput);

            System.out.println("|" + xInput + "|" + xLinearOutput + "|" + xExponentialOutput + "|");

            xInput = xInput + xDelta;
        }
    }

    public void testExponentialyScaleInput() throws Exception {

        int xSteps = 20;
        float xMin = -1;
        float xMax = 1;
        float xRange = xMax - xMin;
        float xDelta = xRange / xSteps;

        float xInput = xMin;
        for (int i = 0; i < xSteps; i++) {

            double xLinearOutput = InputScalerUtils.linearlyScaleInput(xInput);
            double xExponentialOutput = InputScalerUtils.exponentialyScaleInput(xInput);

            System.out.println("|" + xInput + "|" + xLinearOutput + "|" + xExponentialOutput + "|");

            xInput = xInput + xDelta;
        }
    }

    public void testLinearlyScaleInput() throws Exception {

        int xSteps = 20;
        float xMin = -1;
        float xMax = 1;
        float xRange = xMax - xMin;
        float xDelta = xRange / xSteps;

        float xInput = xMin;
        for (int i = 0; i < xSteps; i++) {

            double xLinearOutput = InputScalerUtils.linearlyScaleInput(xInput);
            double xExponentialOutput = InputScalerUtils.exponentialyScaleInput(xInput);

            System.out.println("|" + xInput + "|" + xLinearOutput + "|" + xExponentialOutput + "|");

            xInput = xInput + xDelta;
        }
    }

}