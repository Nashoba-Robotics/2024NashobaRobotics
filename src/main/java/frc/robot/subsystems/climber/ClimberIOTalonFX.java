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
    private TalonFX rightClimber;

    private Servo leftServo;
    private Servo rightServo;

    private TalonFXConfiguration config;
    private MotionMagicDutyCycle leftMotionMagic;
    private MotionMagicDutyCycle rightMotionMagic;

    public ClimberIOTalonFX(){
        leftClimber = new TalonFX(Constants.Climber.LEFT_CLIMBER_PORT, Constants.Climber.CANBUS);
        rightClimber = new TalonFX(Constants.Climber.RIGHT_CLIMBER_PORT, Constants.Climber.CANBUS);

        leftServo = new Servo(Constants.Arm.LEFT_SERVO_CHANNEL);
        rightServo = new Servo(Constants.Arm.RIGHT_SERVO_CHANNEL);

        config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = true;
        config.Audio.BeepOnConfig = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit = Constants.Climber.STATOR_LIMIT;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = Constants.Climber.GEAR_RATIO;

        config.FutureProofConfigs = true;
        config.MotorOutput.Inverted = Constants.Climber.leftInvert;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.FORWARD_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climber.REVERSE_SOFT_LIMIT;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = Constants.Climber.ACCELERATION;
        
        config.Slot0 = Constants.Climber.leftPID;

        rightClimber.setControl(new Follower(Constants.Climber.LEFT_CLIMBER_PORT, true));
        leftClimber.getConfigurator().apply(config);
        rightClimber.getConfigurator().apply(config);

        //TODO: Individually tune the motors;
        // //TODO: MAKE SURE THIS WORKS!!!!
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // config.Slot0 = Constants.Climber.rightPID;
        
        // rightClimber.getConfigurator().apply(config);
       // rightClimber.setControl(new Follower(0, true)); //TODO: Figure out if the motors oppose each other

        leftMotionMagic = new MotionMagicDutyCycle(0, true, 0, 0, true, false, false);
        // rightMotionMagic = new MotionMagicDutyCycle(0, true, 0, 0, true, true, true);
    }

    public void updateInputs(ClimberIOInputs inputs){
        inputs.leftClimberRotorPos = leftClimber.getPosition().getValueAsDouble()*Constants.TAU;
        inputs.leftClimberSpeed = leftClimber.getRotorVelocity().getValueAsDouble()*Constants.TAU;
        inputs.leftClimberStatorCurrent = leftClimber.getStatorCurrent().getValueAsDouble();
        inputs.leftClimberVoltage = leftClimber.getMotorVoltage().getValueAsDouble();

        inputs.rightClimberRotorPos = rightClimber.getPosition().getValueAsDouble()*Constants.TAU;
        inputs.rightClimberSpeed = rightClimber.getRotorVelocity().getValueAsDouble()*Constants.TAU;
        inputs.rightClimberStatorCurrent = rightClimber.getStatorCurrent().getValueAsDouble();
        inputs.rightClimberVoltage = rightClimber.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setLeftClimberPos(Rotation2d pos){
        leftMotionMagic.Position = pos.getRotations();
        leftClimber.setControl(leftMotionMagic);
    }

    @Override
    public void setRightClimberPos(double pos){
        rightMotionMagic.Position = pos;
        rightClimber.setControl(rightMotionMagic);
    }

    public void setLeftClimberRotor(Rotation2d pos){
        leftClimber.setPosition(pos.getRotations());
    }

    public void setRightClimberRotor(Rotation2d pos){
        rightClimber.setPosition(pos.getRotations());
    }

    public void setServo(double pos) {
        leftServo.set(pos);
        rightServo.set(pos);
    }

    @Override
    public void setClimberSpeed(double speed){
        leftClimber.set(speed);
    }

    public void setkS(double kS){
        config.Slot0.kS = kS;
        leftClimber.getConfigurator().apply(config);
        rightClimber.getConfigurator().apply(config);
    }
    public void setkV(double kV){
        config.Slot0.kV = kV;
        leftClimber.getConfigurator().apply(config);
        rightClimber.getConfigurator().apply(config);
    }
    public void setkP(double kP){
        config.Slot0.kP = kP;
        leftClimber.getConfigurator().apply(config);
        rightClimber.getConfigurator().apply(config);
    }
    public void setKD(double kD){
        config.Slot0.kD = kD;
        leftClimber.getConfigurator().apply(config);
        rightClimber.getConfigurator().apply(config);
    }
}
