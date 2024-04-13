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

    public void setSpeed(double speed){
        io.setClimberSpeed(speed);
    }

    public void setPosMeters(double m){
        //setClimberPos(SOME CONVERSION);
    }

<<<<<<< HEAD
    public void setClimberPos(Rotation2d pos){
        io.setLeftClimberPos(pos);
        // io.setRightClimberPos(pos);
    }

    public void setLeftClimberPos(Rotation2d pos){
        io.setLeftClimberPos(pos);
    }
    // public void setRightClimberPos(Rotation2d pos){
    //     io.setRightClimberPos(pos);
    // }

    public void setLeftRotor(Rotation2d pos){
        io.setLeftClimberRotor(pos);
    }
    // public void setRightRotor(Rotation2d pos){
    //     io.setRightClimberRotor(pos);
    // }

    public Rotation2d getLeftClimberPos(){
        return Rotation2d.fromRadians(inputs.leftClimberRotorPos);
    }
    // public Rotation2d getRightClibmerPos(){
    //     return Rotation2d.fromRadians(inputs.rightClimberRotorPos);
    // }
=======
    public void setPos(Rotation2d pos){
        io.setClimberPos(pos);
    }

    public void setRotor(Rotation2d pos){
        io.setClimberRotor(pos);
    }
>>>>>>> bfd02cea78a316241c4434205d8cb26349dda1b7


    public Rotation2d getPos(){
        return Rotation2d.fromRadians(inputs.climberRotorPos);
    }
    public Rotation2d getSpeed(){
        return Rotation2d.fromRadians(inputs.climberSpeed);
    }
    public double getStatorCurrent(){
        return inputs.climberStatorCurrent;
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
