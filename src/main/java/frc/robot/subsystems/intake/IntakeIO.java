package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeSpeed = 0;  //rad/s
        public double intakeTorqueCurrent = 0;  //Amps
        public double intakeStatorCurrent = 0;  //Amps
        public double intakeSupplyCurrent = 0; //Amps
        public double intakeVoltage = 0;    //Volts

        public boolean intakeSensor = false;
    }
    public default void updateInputs(IntakeIOInputs inputs){}

    public abstract void setSpeed(double speed);
    public abstract double getSpeed();
}
