package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO{
    private TalonFX shooter, shooter2;
    private TalonFX pivot;

    private TalonFXConfiguration shooterConfig, pivotConfig;
    private TalonFXConfigurator shooterConfigurator, pivotConfigurator;

    private VelocityDutyCycle shooterControl;
    private MotionMagicDutyCycle pivotControl;

    public ArmIOTalonFX(){
        shooter = new TalonFX(Constants.Arm.SHOOTER_PORT, Constants.Arm.CANBUS);
        shooter2 = new TalonFX(Constants.Arm.SHOOTER_PORT_2, Constants.Arm.CANBUS);

        Follower follower = new Follower(Constants.Arm.SHOOTER_PORT, false);
        shooter2.setControl(follower);

        shooterConfig = new TalonFXConfiguration();
        shooterConfigurator = shooter.getConfigurator();

        pivot = new TalonFX(Constants.Arm.PIVOT_PORT, Constants.Arm.CANBUS);

        pivotConfig = new TalonFXConfiguration();
        pivotConfigurator = pivot.getConfigurator();

        shooterControl = new VelocityDutyCycle(0);
        pivotControl = new MotionMagicDutyCycle(0);

        config();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        inputs.pivotRotorPosition = pivot.getPosition().getValueAsDouble()*Constants.TAU;
        inputs.pivotSpeed = pivot.getVelocity().getValue()*Constants.TAU;
        inputs.pivotStatorCurrent = pivot.getStatorCurrent().getValueAsDouble();
        inputs.pivotSupplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
        inputs.pivotVoltage = pivot.getMotorVoltage().getValueAsDouble();

        inputs.topShooterPosition = shooter.getPosition().getValueAsDouble()*Constants.TAU;
        inputs.topShooterSpeed = shooter.getVelocity().getValueAsDouble()*Constants.TAU;
        inputs.topShooterStatorCurrent = shooter.getStatorCurrent().getValueAsDouble();
        inputs.topShooterSupplyCurrent = shooter.getSupplyCurrent().getValueAsDouble();
        inputs.topShooterVoltage = shooter.getMotorVoltage().getValueAsDouble();

        inputs.bottomShooterPosition = shooter2.getPosition().getValueAsDouble()*Constants.TAU;
        inputs.bottomShooterSpeed = shooter2.getVelocity().getValueAsDouble()*Constants.TAU;
        inputs.bottomShooterStatorCurrent = shooter2.getStatorCurrent().getValueAsDouble();
        inputs.bottomShooterSupplyCurrent = shooter2.getSupplyCurrent().getValueAsDouble();
        inputs.bottomShooterVoltage = shooter2.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setAngle(Rotation2d angle){
        pivotControl.Position = angle.getRotations();
        pivot.setControl(pivotControl);
    }
    @Override
    public void setPivotRotorPos(Rotation2d pos){
        pivot.setPosition(pos.getRotations());
    }

    @Override
    public void setShooterSpeed(Rotation2d speed){
        shooterControl.Velocity = speed.getRotations();
        shooter.setControl(shooterControl);
    }

    public void setPivotSpeed(double speed){
        pivot.set(speed);
    }
    
    public void setPivotkG(double kG){
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.kG = kG;
        pivot.getConfigurator().apply(pivotConfig);
    }
    public void setPivotkS(double kS){
        pivotConfig.Slot0.kS = kS;
        pivot.getConfigurator().apply(pivotConfig);
    }
    public void setPivotkV(double kV){
        pivotConfig.Slot0.kV = kV;
        pivot.getConfigurator().apply(pivotConfig);
    }
    public void setPivotkP(double kP){
        pivotConfig.Slot0.kP = kP;
        pivot.getConfigurator().apply(pivotConfig);
    }
    public void setPivotkD(double kD){
        pivotConfig.Slot0.kD = kD;
        pivot.getConfigurator().apply(pivotConfig);
    }

    public void setShooterPercent(double percent) {
        shooter.set(percent);
    }

    public TalonFX getShooterMotor() {
        return shooter;
    }

    private void config() {
        pivotConfig.Audio.BeepOnBoot = true;
        pivotConfig.Audio.BeepOnConfig = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Arm.PIVOT_STATOR_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.Arm.PIVOT_SUPPLY_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.PIVOT_MOTION_MAGIC_ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.PIVOT_MOTION_MAGIC_CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicJerk = Constants.Arm.PIVOT_MOTION_MAGIC_JERK;
        pivotConfig.MotorOutput.Inverted = Constants.Arm.PIVOT_INVERTED;
        pivotConfig.MotorOutput.NeutralMode = Constants.Arm.PIVOT_NEUTRAL_MODE;
        pivotConfig.Slot0 = Constants.Arm.PIVOT_PID;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.PIVOT_FORWARD_SOFT_LIMIT.getRotations();
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.PIVOT_REVERSE_SOFT_LIMIT.getRotations();
        pivotConfig.Voltage.PeakForwardVoltage = Constants.PEAK_VOLTAGE;
        pivotConfig.Voltage.PeakReverseVoltage = -Constants.PEAK_VOLTAGE;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Arm.PIVOT_GEAR_RATIO;

        shooterConfig.Audio.BeepOnBoot = true;
        shooterConfig.Audio.BeepOnConfig = true;
        shooterConfig.CurrentLimits.StatorCurrentLimit = Constants.Arm.SHOOTER_STATOR_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = Constants.Arm.SHOOTER_SUPPLY_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        shooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        shooterConfig.MotorOutput.Inverted = Constants.Arm.SHOOTER_INVERTED;
        shooterConfig.MotorOutput.NeutralMode = Constants.Arm.SHOOTER_NEUTRAL_MODE;
        shooterConfig.Slot0 = Constants.Arm.SHOOTER_PID;
        shooterConfig.Voltage.PeakForwardVoltage = Constants.PEAK_VOLTAGE;
        shooterConfig.Voltage.PeakReverseVoltage = -Constants.PEAK_VOLTAGE;
        shooterConfig.Feedback.SensorToMechanismRatio = Constants.Arm.SHOOTER_GEAR_RATIO;

        pivotConfigurator.apply(pivotConfig);
        shooterConfigurator.apply(shooterConfig);
        shooter2.getConfigurator().apply(shooterConfig);
    }
}
