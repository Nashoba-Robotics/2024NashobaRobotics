package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberRotorPos = 0;  //Rad
        public double climberSpeed = 0; //rad/s
        public double climberStatorCurrent = 0; //Amps
        public double climberVoltage = 0;   //Volts
    }

    public default void updateInputs(ClimberIOInputs inputs){};

<<<<<<< HEAD
    public abstract void setLeftClimberPos(Rotation2d pos);
    // public abstract void setRightClimberPos(Rotation2d pos);

    public abstract void setClimberSpeed(double speed);
    
    public abstract void setLeftClimberRotor(Rotation2d pos);
    // public abstract void setRightClimberRotor(Rotation2d pos);

    public void setServo(double pos);
=======
    public abstract void setClimberPos(Rotation2d pos);
    public abstract void setClimberSpeed(double speed);
>>>>>>> bfd02cea78a316241c4434205d8cb26349dda1b7

    public abstract void setClimberRotor(Rotation2d pos);

    public abstract void setkS(double kS);
    public abstract void setkV(double kV);
    public abstract void setkP(double kP);
    public abstract void setKD(double kD);
}
