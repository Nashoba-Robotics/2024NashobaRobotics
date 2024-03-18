package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double leftClimberRotorPos = 0;  //Rad
        public double leftClimberSpeed = 0; //rad/s
        public double leftClimberStatorCurrent = 0; //Amps
        public double leftClimberVoltage = 0;   //Volts

        public double rightClimberRotorPos = 0;  //Rad
        public double rightClimberSpeed = -0;   //rad/s
        public double rightClimberStatorCurrent = 0; //Amps
        public double rightClimberVoltage = 0;   //Volts
    }

    public default void updateInputs(ClimberIOInputs inputs){};

    public abstract void setLeftClimberPos(double pos);
    public abstract void setRightClimberPos(double pos);

    public abstract void setClimberSpeed(double speed);
    
    public abstract void setLeftClimberRotor(Rotation2d pos);
    public abstract void setRightClimberRotor(Rotation2d pos);

    public void setServo(double pos);


    public abstract void setkS(double kS);
    public abstract void setkV(double kV);
    public abstract void setkP(double kP);
    public abstract void setKD(double kD);
}
