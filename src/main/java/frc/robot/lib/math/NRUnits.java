package frc.robot.lib.math;

import frc.robot.Constants;

public class NRUnits {

    // Constrains the given angle to [-180, 180] 
    public static double constrainDeg(double angle){
        return (angle % 360 + 360) % 360 - 180;
    }


    public static double constrainRad(double angle) {
        return (angle % Constants.TAU + Constants.TAU) % Constants.TAU - Constants.TAU/2;
    }
}