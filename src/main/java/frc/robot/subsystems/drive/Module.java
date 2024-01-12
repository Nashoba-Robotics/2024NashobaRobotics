package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class Module {

    public int modIndex;
 
    private ModuleIO io;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + modIndex, inputs);
    }
    
    public Module(int modIndex, String canBus) {
        this.modIndex = modIndex;

        SwerveModuleConstants constants;
        
        switch (modIndex) {
            case 0:
                constants = Constants.Drive.MOD0_CONSTANTS;
                break;
            case 1:
                constants = Constants.Drive.MOD1_CONSTANTS;
                break;
            case 2:
                constants = Constants.Drive.MOD2_CONSTANTS;
                break;
            case 3:
                constants = Constants.Drive.MOD3_CONSTANTS;
                break;
            
            default:
                constants = new SwerveModuleConstants();
                break;
        }
        
        io = new ModuleIOTalonFX(constants, canBus);
    }

    public void set(SwerveModuleState state) {
        io.set(state);
    }

    public SwerveModulePosition getPosition() {
        return io.getPosition();
    }

    public SwerveModuleState getState() {
        return io.getState();
    }

}
