package frc.robot.commands.test;


import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class FindLoaderZero extends Command {
    
    private LoaderSubsystem loader;

    Timer t;

    public FindLoaderZero(LoaderSubsystem loader) {
        this.loader = loader;
        addRequirements(loader);

        t = new Timer();
    }

    @Override
    public void initialize() {
        TalonFXConfiguration config = loader.getPivotConfig();
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        loader.setPivotConfig(config);
        t.restart();
    }

    @Override
    public void execute() {
        loader.setPivotSpeed(-0.1);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(loader.getPivotCurrent()) > 10;
    }

    @Override
    public void end(boolean interrupted) {
        loader.setPivotSpeed(0);
        loader.setPivotRotor(Rotation2d.fromRadians(0));
        TalonFXConfiguration config = loader.getPivotConfig();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        loader.setPivotConfig(config);
    }

}
