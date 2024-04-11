package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO{
    private TalonFX clibmer;

    private TalonFXConfiguration climberConfig;
    private MotionMagicDutyCycle climbMotionMagic;

    public ClimberIOTalonFX(){
        clibmer = new TalonFX(Constants.Climber.CLIMBER_PORT, Constants.Climber.CANBUS);

        climberConfig = new TalonFXConfiguration();
        climberConfig.Audio.BeepOnBoot = true;
        climberConfig.Audio.BeepOnConfig = false;
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        climberConfig.CurrentLimits.StatorCurrentLimit = Constants.Climber.STATOR_LIMIT;

        climberConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        climberConfig.Feedback.SensorToMechanismRatio = Constants.Climber.GEAR_RATIO;

        climberConfig.FutureProofConfigs = true;
        climberConfig.MotorOutput.Inverted = Constants.Climber.leftInvert;
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberConfig.Voltage.PeakForwardVoltage = 12;
        climberConfig.Voltage.PeakReverseVoltage = -12;
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.FORWARD_SOFT_LIMIT;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climber.REVERSE_SOFT_LIMIT;

        climberConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.CRUISE_VELOCITY;
        climberConfig.MotionMagic.MotionMagicAcceleration = Constants.Climber.ACCELERATION;
        
        climberConfig.Slot0 = Constants.Climber.pid;
        
        clibmer.getConfigurator().apply(climberConfig);
        
        climbMotionMagic = new MotionMagicDutyCycle(0, true, 0, 0, true, false, false);
    }

    public void updateInputs(ClimberIOInputs inputs){
        inputs.climberRotorPos = clibmer.getPosition().getValueAsDouble()*Constants.TAU;
        inputs.climberSpeed = clibmer.getRotorVelocity().getValueAsDouble()*Constants.TAU;
        inputs.climberStatorCurrent = clibmer.getStatorCurrent().getValueAsDouble();
        inputs.climberVoltage = clibmer.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setClimberPos(Rotation2d pos){
        climbMotionMagic.Position = pos.getRotations();
        clibmer.setControl(climbMotionMagic);
    }


    public void setClimberRotor(Rotation2d pos){
        clibmer.setPosition(pos.getRotations());
    }


    @Override
    public void setClimberSpeed(double speed){
        clibmer.set(speed);
    }

    public void setkS(double kS){
        climberConfig.Slot0.kS = kS;
        clibmer.getConfigurator().apply(climberConfig);
    }
    public void setkV(double kV){
        climberConfig.Slot0.kV = kV;
        clibmer.getConfigurator().apply(climberConfig);
    }
    public void setkP(double kP){
        climberConfig.Slot0.kP = kP;
        clibmer.getConfigurator().apply(climberConfig);
    }
    public void setKD(double kD){
        climberConfig.Slot0.kD = kD;
        clibmer.getConfigurator().apply(climberConfig);
    }
}
