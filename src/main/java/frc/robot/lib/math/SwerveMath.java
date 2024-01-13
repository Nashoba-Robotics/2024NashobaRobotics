package frc.robot.lib.math;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveMath {
    
    //if magnitudes are greater than 1, will normalize all the states so that the max is 1
    public static SwerveModuleState[] normalize(SwerveModuleState[] states) {

        double max = 0;
        for(SwerveModuleState state : states) {
            max = state.speedMetersPerSecond > max ? state.speedMetersPerSecond : max;
        }

        if(max > Constants.Drive.MAX_VELOCITY) {
            for(SwerveModuleState state : states) {
                state.speedMetersPerSecond /= max;
            }
        }

        return states;
    }

}
