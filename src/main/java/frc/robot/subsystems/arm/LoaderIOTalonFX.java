package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class LoaderIOTalonFX implements LoaderIO {
    
    private TalonFX pivot;
    private TalonFXConfiguration pivotConfig;
    private TalonFXConfigurator pivotConfigurator;
    private MotionMagicDutyCycle pivotController;

    // private TalonFX roller;
    private TalonFXConfiguration rollerConfig;
    private TalonFXConfigurator rollerConfigurator;
    private VelocityDutyCycle rollerController;

    public LoaderIOTalonFX() {
        pivot = new TalonFX(Constants.Loader.PIVOT_PORT, Constants.Loader.CANBUS);
        pivotConfig = new TalonFXConfiguration();
        pivotConfigurator = pivot.getConfigurator();
        pivotController = new MotionMagicDutyCycle(0);

        // roller = new TalonFX(Constants.Loader.ROLLER_PORT, Constants.Loader.CANBUS);
        // rollerConfig = new TalonFXConfiguration();
        // rollerConfigurator = roller.getConfigurator();
        // rollerController = new VelocityDutyCycle(0);

        config();
    }

    public void setPivotPosition(Rotation2d pos) {
        pivotController.Position = pos.getRotations();

        pivot.setControl(pivotController);
    }

    public void setPivotRotorPos(Rotation2d pos){
        pivot.setPosition(pos.getRotations());
    }

    public void setRollerSpeed(Rotation2d speed) {
        // rollerController.Velocity = speed.getRotations();

        // roller.setControl(rollerController);
    }

    public void setLoaderkV(double kV){
        pivotConfig.Slot0.kV = kV;
        pivotConfigurator.apply(pivotConfig);
    }
    public void setLoaderkP(double kP){
        pivotConfig.Slot0.kP = kP;
        pivotConfigurator.apply(pivotConfig);
    }
    public void setLoaderkD(double kD){
        pivotConfig.Slot0.kD = kD;
        pivotConfigurator.apply(pivotConfig);
    }
    public void setPivotSpeed(double speed){
        pivot.set(speed);
    }

    public void updateInputs(LoaderIOInputs inputs) {
        inputs.pivotPosition = pivot.getPosition().getValueAsDouble() * Constants.TAU;
        inputs.pivotVelocity = pivot.getVelocity().getValueAsDouble() * Constants.TAU;
        inputs.pivotSupplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
        inputs.pivotStatorCurrent = pivot.getStatorCurrent().getValueAsDouble();
        inputs.pivotVoltage = pivot.getMotorVoltage().getValueAsDouble();

        // inputs.rollerPosition = roller.getPosition().getValueAsDouble() * Constants.TAU;
        // inputs.rollerVelocity = roller.getVelocity().getValueAsDouble() * Constants.TAU;
        // inputs.rollerSupplyCurrent = roller.getSupplyCurrent().getValueAsDouble();
        // inputs.rollerStatorCurrent = roller.getStatorCurrent().getValueAsDouble();
        // inputs.rollerVoltage = roller.getMotorVoltage().getValueAsDouble();
    }

    private void config() { //TODO: Make Current limits true
        pivotConfig.Audio.BeepOnBoot = true;
        pivotConfig.Audio.BeepOnConfig = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Loader.PIVOT_STATOR_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.Loader.PIVOT_SUPPLY_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Loader.PIVOT_GEAR_RATIO;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Constants.Loader.PIVOT_MOTION_MAGIC_ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Loader.PIVOT_MOTION_MAGIC_CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicJerk = Constants.Loader.PIVOT_MOTION_MAGIC_JERK;
        pivotConfig.MotorOutput.Inverted = Constants.Loader.PIVOT_INVERTED;
        pivotConfig.MotorOutput.NeutralMode = Constants.Loader.PIVOT_NEUTRAL_MODE;
        pivotConfig.Slot0 = Constants.Loader.PIVOT_PID;
        pivotConfig.Voltage.PeakForwardVoltage = Constants.PEAK_VOLTAGE;
        pivotConfig.Voltage.PeakReverseVoltage = -Constants.PEAK_VOLTAGE;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.Loader.PIVOT_GEAR_RATIO;

        // rollerConfig.Audio.BeepOnBoot = true;
        // rollerConfig.Audio.BeepOnConfig = true;
        // rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Loader.ROLLER_STATOR_CURRENT_LIMIT;
        // rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Loader.ROLLER_SUPPLY_CURRENT_LIMIT;
        // rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // rollerConfig.Feedback.SensorToMechanismRatio = Constants.Loader.ROLLER_GEAR_RATIO;
        // rollerConfig.MotionMagic.MotionMagicAcceleration = Constants.Loader.ROLLER_MOTION_MAGIC_ACCELERATION;
        // rollerConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Loader.ROLLER_MOTION_MAGIC_CRUISE_VELOCITY;
        // rollerConfig.MotionMagic.MotionMagicJerk = Constants.Loader.ROLLER_MOTION_MAGIC_JERK;
        // rollerConfig.MotorOutput.Inverted = Constants.Loader.ROLLER_INVERTED;
        // rollerConfig.MotorOutput.NeutralMode = Constants.Loader.ROLLER_NEUTRAL_MODE;
        // rollerConfig.Slot0 = Constants.Loader.ROLLER_PID;
        // rollerConfig.Voltage.PeakForwardVoltage = Constants.PEAK_VOLTAGE;
        // rollerConfig.Voltage.PeakReverseVoltage = -Constants.PEAK_VOLTAGE;
        // rollerConfig.Feedback.SensorToMechanismRatio = Constants.Loader.ROLLER_GEAR_RATIO;

        pivotConfigurator.apply(pivotConfig);
        // rollerConfigurator.apply(rollerConfig);
    }

}
