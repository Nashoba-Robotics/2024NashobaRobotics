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
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO{
    // //2 for shooter rollers (Krakens)
    // //1 for pivot (Kraken maybe)
    private TalonFX shooter, shooter2;
    private TalonFX pivot;

    private TalonFXConfiguration shooterConfig, pivotConfig;
    private TalonFXConfigurator shooterConfigurator, pivotConfigurator;

    private VelocityDutyCycle shooterControl;
    private MotionMagicDutyCycle pivotControl;

    private DigitalInput shooterSensor, loaderSensor;

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

        shooterSensor = new DigitalInput(Constants.Arm.SHOOTER_SENSOR_PORT);
        loaderSensor = new DigitalInput(Constants.Arm.SHOOTER_SENSOR_PORT);

        shooterControl = new VelocityDutyCycle(0);
        pivotControl = new MotionMagicDutyCycle(0);

        config();
    }

    public void updateInputs(ArmIOInputs inputs){
        inputs.pivotRotorPosition = pivot.getPosition().getValueAsDouble();
        inputs.pivotSpeed = pivot.getVelocity().getValue();
        inputs.pivotStatorCurrent = pivot.getStatorCurrent().getValueAsDouble();
        inputs.pivotSupplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
        inputs.pivotVoltage = pivot.getMotorVoltage().getValueAsDouble();

        inputs.shooterPosition = shooter.getPosition().getValueAsDouble();
        inputs.shooterSpeed = shooter.getVelocity().getValueAsDouble();
        inputs.shooterStatorCurrent = shooter.getStatorCurrent().getValueAsDouble();
        inputs.shooterSupplyCurrent = shooter.getSupplyCurrent().getValueAsDouble();
        inputs.shooterVoltage = shooter.getMotorVoltage().getValueAsDouble();

        inputs.shooterSensor = shooterSensor.get();
        inputs.loaderSensor = loaderSensor.get();
    }

    @Override
    public void setAngle(Rotation2d angle){
        pivotControl.Position = angle.getRotations();
        pivot.setControl(pivotControl);
    }

    @Override
    public void setShooterSpeed(Rotation2d speed){
        shooterControl.Velocity = speed.getRotations();
        shooter.setControl(shooterControl);
    }

    public boolean getShooterSensor() {
        return shooterSensor.get();
    }

    public boolean getLoaderSensor() {
        return loaderSensor.get();
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
        pivotConfig.Voltage.PeakForwardVoltage = Constants.PEAK_VOLTAGE;
        pivotConfig.Voltage.PeakReverseVoltage = -Constants.PEAK_VOLTAGE;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Arm.PIVOT_GEAR_RATIO;

        shooterConfig.Audio.BeepOnBoot = true;
        shooterConfig.Audio.BeepOnConfig = true;
        shooterConfig.CurrentLimits.StatorCurrentLimit = Constants.Arm.SHOOTER_STATOR_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = Constants.Arm.SHOOTER_SUPPLY_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        shooterConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.SHOOTER_MOTION_MAGIC_ACCELERATION;
        shooterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.SHOOTER_MOTION_MAGIC_CRUISE_VELOCITY;
        shooterConfig.MotionMagic.MotionMagicJerk = Constants.Arm.SHOOTER_MOTION_MAGIC_JERK;
        shooterConfig.MotorOutput.Inverted = Constants.Arm.SHOOTER_INVERTED;
        shooterConfig.MotorOutput.NeutralMode = Constants.Arm.SHOOTER_NEUTRAL_MODE;
        shooterConfig.Slot0 = Constants.Arm.SHOOTER_PID;
        shooterConfig.Voltage.PeakForwardVoltage = Constants.PEAK_VOLTAGE;
        shooterConfig.Voltage.PeakReverseVoltage = -Constants.PEAK_VOLTAGE;
        shooterConfig.Feedback.SensorToMechanismRatio = Constants.Arm.SHOOTER_GEAR_RATIO;

        pivotConfigurator.apply(pivotConfig);
        shooterConfigurator.apply(shooterConfig);
    }
}
