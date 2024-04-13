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
    private TalonFX leftClimber;
    // private TalonFX rightClimber;

    private Servo leftServo;
    private Servo rightServo;

    private TalonFXConfiguration leftConfig;
    private TalonFXConfiguration rightConfig;
    private MotionMagicDutyCycle leftMotionMagic;
    private MotionMagicDutyCycle rightMotionMagic;

    public ClimberIOTalonFX(){
        leftClimber = new TalonFX(Constants.Climber.LEFT_CLIMBER_PORT, Constants.Climber.CANBUS);
        // rightClimber = new TalonFX(Constants.Climber.RIGHT_CLIMBER_PORT, Constants.Climber.CANBUS);

        leftServo = new Servo(Constants.Arm.LEFT_SERVO_CHANNEL);
        rightServo = new Servo(Constants.Arm.RIGHT_SERVO_CHANNEL);

        leftConfig = new TalonFXConfiguration();
        leftConfig.Audio.BeepOnBoot = true;
        leftConfig.Audio.BeepOnConfig = false;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        leftConfig.CurrentLimits.StatorCurrentLimit = Constants.Climber.STATOR_LIMIT;

        leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftConfig.Feedback.SensorToMechanismRatio = Constants.Climber.GEAR_RATIO;

        leftConfig.FutureProofConfigs = true;
        leftConfig.MotorOutput.Inverted = Constants.Climber.leftInvert;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.Voltage.PeakForwardVoltage = 12;
        leftConfig.Voltage.PeakReverseVoltage = -12;
        leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.FORWARD_SOFT_LIMIT;
        leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climber.REVERSE_SOFT_LIMIT;

        leftConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.CRUISE_VELOCITY;
        leftConfig.MotionMagic.MotionMagicAcceleration = Constants.Climber.ACCELERATION;
        
        leftConfig.Slot0 = Constants.Climber.leftPID;
        //*********************** */
        rightConfig = new TalonFXConfiguration();
        rightConfig.Audio.BeepOnBoot = true;
        rightConfig.Audio.BeepOnConfig = false;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        rightConfig.CurrentLimits.StatorCurrentLimit = Constants.Climber.STATOR_LIMIT;

        rightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightConfig.Feedback.SensorToMechanismRatio = Constants.Climber.GEAR_RATIO;

        rightConfig.FutureProofConfigs = true;
        rightConfig.MotorOutput.Inverted = Constants.Climber.rightInvert;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.Voltage.PeakForwardVoltage = 12;
        rightConfig.Voltage.PeakReverseVoltage = -12;
        rightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        rightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.FORWARD_SOFT_LIMIT;
        rightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        rightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climber.REVERSE_SOFT_LIMIT;

        rightConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.CRUISE_VELOCITY;
        rightConfig.MotionMagic.MotionMagicAcceleration = Constants.Climber.ACCELERATION;
        
        rightConfig.Slot0 = Constants.Climber.rightPID;

        leftClimber.getConfigurator().apply(leftConfig);
        
        // rightClimber.getConfigurator().apply(rightConfig);

        leftMotionMagic = new MotionMagicDutyCycle(0, true, 0, 0, true, false, false);
        rightMotionMagic = new MotionMagicDutyCycle(0, true, 0, 0, true, false, false);
    }

    public void updateInputs(ClimberIOInputs inputs){
        inputs.leftClimberRotorPos = leftClimber.getPosition().getValueAsDouble()*Constants.TAU;
        inputs.leftClimberSpeed = leftClimber.getRotorVelocity().getValueAsDouble()*Constants.TAU;
        inputs.leftClimberStatorCurrent = leftClimber.getStatorCurrent().getValueAsDouble();
        inputs.leftClimberVoltage = leftClimber.getMotorVoltage().getValueAsDouble();

        // inputs.rightClimberRotorPos = rightClimber.getPosition().getValueAsDouble()*Constants.TAU;
        // inputs.rightClimberSpeed = rightClimber.getRotorVelocity().getValueAsDouble()*Constants.TAU;
        // inputs.rightClimberStatorCurrent = rightClimber.getStatorCurrent().getValueAsDouble();
        // inputs.rightClimberVoltage = rightClimber.getMotorVoltage().getValueAsDouble();

        inputs.leftServoPos = leftServo.getPosition();
        inputs.rightServoPos = rightServo.getPosition();
    }

    @Override
    public void setLeftClimberPos(Rotation2d pos){
        leftMotionMagic.Position = pos.getRotations();
        leftClimber.setControl(leftMotionMagic);
    }

    // @Override
    // public void setRightClimberPos(Rotation2d pos){
    //     rightMotionMagic.Position = pos.getRotations();
    //     rightClimber.setControl(rightMotionMagic);
    // }

    public void setLeftClimberRotor(Rotation2d pos){
        leftClimber.setPosition(pos.getRotations());
    }

    // public void setRightClimberRotor(Rotation2d pos){
    //     rightClimber.setPosition(pos.getRotations());
    // }

    public void setServo(double pos) {
        leftServo.set(pos);
        rightServo.set(pos);
    }

    @Override
    public void setClimberSpeed(double speed){
        leftClimber.set(speed);
        // rightClimber.set(speed);
    }

    public void setkS(double kS){
        leftConfig.Slot0.kS = kS;
        rightConfig.Slot0.kS = kS;
        leftClimber.getConfigurator().apply(leftConfig);
        // rightClimber.getConfigurator().apply(rightConfig);
    }
    public void setkV(double kV){
        leftConfig.Slot0.kV = kV;
        rightConfig.Slot0.kV = kV;
        leftClimber.getConfigurator().apply(leftConfig);
        // rightClimber.getConfigurator().apply(rightConfig);
    }
    public void setkP(double kP){
        leftConfig.Slot0.kP = kP;
        rightConfig.Slot0.kP = kP;
        leftClimber.getConfigurator().apply(leftConfig);
        // rightClimber.getConfigurator().apply(rightConfig);
    }
    public void setKD(double kD){
        leftConfig.Slot0.kD = kD;
        rightConfig.Slot0.kD = kD;
        leftClimber.getConfigurator().apply(leftConfig);
        // rightClimber.getConfigurator().apply(rightConfig);
    }
}
