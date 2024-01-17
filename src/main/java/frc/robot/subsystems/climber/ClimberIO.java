package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.annotation.JsonAppend.Attr;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{
        public double climberRotorPos = 0;  //Rot
        public double climberStatorCurrent = 0; //Amps
        public double climberVoltage = 0;   //Volts
    }

    public abstract void setClimberPos(double pos);
}
