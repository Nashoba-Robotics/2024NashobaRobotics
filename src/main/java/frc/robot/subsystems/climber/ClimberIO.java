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

    public abstract void setClimberPos(Rotation2d pos);
    public abstract void setClimberSpeed(double speed);

    public abstract void setClimberRotor(Rotation2d pos);

    public abstract void enableReverseSoftLimit(boolean limit);

    public abstract void setkS(double kS);
    public abstract void setkV(double kV);
    public abstract void setkP(double kP);
    public abstract void setKD(double kD);
}
