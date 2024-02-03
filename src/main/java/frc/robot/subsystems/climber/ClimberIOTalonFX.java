package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO{
    private TalonFX leftClimber;
    private TalonFX rightClimber;

    private TalonFXConfiguration config;
    private MotionMagicDutyCycle leftMotionMagic;
    private MotionMagicDutyCycle rightMotionMagic;

    public ClimberIOTalonFX(){
        leftClimber = new TalonFX(Constants.Climber.LEFT_CLIMBER_PORT, Constants.Climber.CANBUS);
        rightClimber = new TalonFX(Constants.Climber.RIGHT_CLIMBER_PORT, Constants.Climber.CANBUS);

        config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = true;
        config.Audio.BeepOnConfig = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit = Constants.Climber.STATOR_LIMIT;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = Constants.Climber.GEAR_RATIO;
        config.FutureProofConfigs = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.FORWARD_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climber.REVERSE_SOFT_LIMIT;
        
        config.Slot0 = Constants.Climber.leftPID;

        leftClimber.getConfigurator().apply(config);

        //TODO: MAKE SURE THIS WORKS!!!!
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Slot0 = Constants.Climber.rightPID;
        
        rightClimber.getConfigurator().apply(config);
       // rightClimber.setControl(new Follower(0, true)); //TODO: Figure out if the motors oppose each other

        leftMotionMagic = new MotionMagicDutyCycle(0, true, 0, 0, true, true, true);
        rightMotionMagic = new MotionMagicDutyCycle(0, true, 0, 0, true, true, true);
    }

    public void updateInputs(ClimberIOInputs inputs){
        inputs.leftClimberRotorPos = leftClimber.getRotorPosition().getValueAsDouble()*Constants.TAU;
        inputs.leftClimberSpeed = leftClimber.getRotorVelocity().getValueAsDouble()*Constants.TAU;
        inputs.leftClimberStatorCurrent = leftClimber.getStatorCurrent().getValueAsDouble();
        inputs.leftClimberVoltage = leftClimber.getMotorVoltage().getValueAsDouble();

        inputs.rightClimberRotorPos = rightClimber.getRotorPosition().getValueAsDouble()*Constants.TAU;
        inputs.rightClimberSpeed = rightClimber.getRotorVelocity().getValueAsDouble()*Constants.TAU;
        inputs.rightClimberStatorCurrent = rightClimber.getStatorCurrent().getValueAsDouble();
        inputs.rightClimberVoltage = rightClimber.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setLeftClimberPos(double pos){
        leftMotionMagic.Position = pos;
        leftClimber.setControl(leftMotionMagic);
    }

    @Override
    public void setRightClimberPos(double pos){
        rightMotionMagic.Position = pos;
        rightClimber.setControl(rightMotionMagic);
    }
}
