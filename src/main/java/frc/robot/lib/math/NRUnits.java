package frc.robot.lib.math;

import frc.robot.Constants;

public class NRUnits {

    // Constrains the given angle to [-180, 180] 
    public static double constrainDeg(double angle){
        return 360./Constants.TAU*constrainRad(angle*Constants.TAU/360);
    }


    public static double constrainRad(double angle) {
        double temp = (angle % Constants.TAU + Constants.TAU) % Constants.TAU;
        if(temp <= Constants.TAU/2) return temp;
        return temp - Constants.TAU;
    }

    public static double logConstrainRad(double angle) {
        angle = constrainRad(angle);
        if(angle < 0) angle += Constants.TAU/2;
        return angle;
    }
}