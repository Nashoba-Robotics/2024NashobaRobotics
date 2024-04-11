package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
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

    public void setClimberSpeed(double speed){
        io.setClimberSpeed(speed);
    }

    public void setClimberM(double m){
        //setClimberPos(SOME CONVERSION);
    }

    public void setClimberPos(Rotation2d pos){
        io.setClimberPos(pos);
    }

    public void setRotor(Rotation2d pos){
        io.setClimberRotor(pos);
    }


    public Rotation2d getClimberPos(){
        return Rotation2d.fromRadians(inputs.climberRotorPos);
    }

    public void setkS(double kS){
        io.setkS(kS);
    }
    public void setkV(double kV){
        io.setkV(kV);
    }
    public void setkP(double kP){
        io.setkP(kP);
    }
    public void setkD(double kD){
        io.setKD(kD);
    }
}
