package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsytem extends SubsystemBase{
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public ClimberSubsytem(){
        io = new ClimberIOTalonFX();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public void setClimberM(double m){
        //setClimberPos(SOME CONVERSION);
    }

    public void setClimberPos(double pos){
        io.setLeftClimberPos(pos);
        io.setRightClimberPos(pos);
    }

    public void setLeftClimberPos(double pos){
        io.setLeftClimberPos(pos);
    }
    public void setRightClimberPos(double pos){
        io.setRightClimberPos(pos);
    }

    public double getLeftRad(double pos){
        return inputs.leftClimberRotorPos;
    }
    public double getRightRad(double pos){
        return inputs.rightClimberRotorPos;
    }
}
