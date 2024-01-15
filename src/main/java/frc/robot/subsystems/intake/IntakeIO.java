package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double intakeSpeed = 0;  //rps
        public double intakeStatorCurrent = 0;  //Amps
        public double intakeSupplyCurrent = 0; //Amps
        public double intakeVoltage = 0;    //Volts
    }

    public abstract void setSpeed();
}
