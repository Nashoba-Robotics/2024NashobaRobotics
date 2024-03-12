package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public interface SensorIO{
    @AutoLog
    public static class SensorIOInputs{
        public boolean shooter1 = false;
        public boolean shooter2 = false;

        public boolean loader = false;

        public boolean intake = false;
    }

    public default void updateInputs(SensorIOInputs inputs){}

}