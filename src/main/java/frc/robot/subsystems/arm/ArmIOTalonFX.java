package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO{
    //2 for shooter rollers (Krakens)
    //1 for pivot (Kraken maybe)
    private TalonFX shooter, shooter2;
    private TalonFX pivot;

    private TalonFXConfiguration shooterConfig, pivotConfig;

    public ArmIOTalonFX(){
        shooter = new TalonFX(Constants.Arm.SHOOTER_PORT, "rio");
        shooter2 = new TalonFX(Constants.Arm.SHOOTER_PORT_2, "rio");

        pivot = new TalonFX(Constants.Arm.PIVOT_PORT, "rio");

        shooterConfig = new TalonFXConfiguration();
        shooterConfig.Audio.BeepOnBoot = true;
        shooterConfig.Audio.BeepOnConfig = false;
        shooterConfig.CurrentLimits.StatorCurrentLimit = 0;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        shooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;



    }

    public void updateInputs(ArmIOInputs inputs){

    }

    @Override
    public void setAngle(double angle){

    }

    @Override
    public void setShooterSpeed(double speed){
        
    }
}
