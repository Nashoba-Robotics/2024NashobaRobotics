package frc.robot.commands.test;


import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class FindLoaderZero extends Command {
    
    private ArmSubsystem arm;

    Timer t;

    public FindLoaderZero(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);

        t = new Timer();
    }

    @Override
    public void initialize() {
        TalonFXConfiguration config = arm.getLoaderPivotConfig();
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        arm.setLoaderPivotConfig(config);
        t.restart();
    }

    @Override
    public void execute() {
        arm.setLoaderPivotSpeed(-0.1);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getLoaderPivotCurrent()) > 10;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setLoaderPivotSpeed(0);
        arm.setLoaderPivotRotor(Rotation2d.fromRadians(0));
        TalonFXConfiguration config = arm.getLoaderPivotConfig();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        arm.setLoaderPivotConfig(config);
    }

}
