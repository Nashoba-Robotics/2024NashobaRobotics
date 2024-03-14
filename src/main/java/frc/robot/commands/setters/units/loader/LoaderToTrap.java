package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class LoaderToTrap extends Command {
    
    private LoaderSubsystem loader = RobotContainer.loader;

    public LoaderToTrap() {
        addRequirements(loader);
    }

    @Override
    public void initialize() {
        loader.setPivot(Presets.Loader.TRAP_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Presets.Loader.TRAP_POS.getRadians() - loader.getPivotAngle().getRadians()) < Presets.Loader.POS_TOLERANCE.getRadians();
    }

}
