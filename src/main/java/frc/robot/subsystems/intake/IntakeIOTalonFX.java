package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO{
    TalonFX intake;
    TalonFXConfiguration config;


    public IntakeIOTalonFX(){
        intake = new TalonFX(8);

        config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = true;
        config.Audio.BeepOnConfig = false;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.RotorToSensorRatio = 0;
        config.FutureProofConfigs = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        intake.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.intakeSpeed = intake.getVelocity().getValueAsDouble()*Constants.TAU;
        inputs.intakeTorqueCurrent = intake.getTorqueCurrent().getValueAsDouble();
        inputs.intakeStatorCurrent = intake.getStatorCurrent().getValueAsDouble();
        inputs.intakeSupplyCurrent = intake.getSupplyCurrent().getValueAsDouble();
        inputs.intakeVoltage = intake.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed){
        intake.set(speed);
    }
    @Override
    public double getSpeed(){
        return intake.getVelocity().getValueAsDouble();
    }
}
