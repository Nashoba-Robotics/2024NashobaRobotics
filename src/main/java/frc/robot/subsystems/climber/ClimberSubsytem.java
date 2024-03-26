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
        io.setLeftClimberPos(pos);
        // io.setRightClimberPos(pos);
    }

    public void setLeftClimberPos(Rotation2d pos){
        io.setLeftClimberPos(pos);
    }
    public void setRightClimberPos(double pos){
        // io.setRightClimberPos(pos);
    }

    public void setLeftRotor(Rotation2d pos){
        io.setLeftClimberRotor(pos);
    }
    public void setRightRotor(Rotation2d pos){
        io.setRightClimberRotor(pos);
    }

    public double getLeftRad(){
        return inputs.leftClimberRotorPos;
    }
    public double getRightRad(){
        return inputs.rightClimberRotorPos;
    }


    public void setServo(double pos) {
        io.setServo(pos);
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
