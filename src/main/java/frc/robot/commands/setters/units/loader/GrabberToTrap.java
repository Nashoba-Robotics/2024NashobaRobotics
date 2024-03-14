package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class GrabberToTrap extends Command {
    
    private LoaderSubsystem loader = RobotContainer.loader;

    public GrabberToTrap() {
        addRequirements(loader);
    }

    @Override
    public void initialize() {
        loader.setRollerSpeed(Presets.Loader.TRAP_SPEED);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
