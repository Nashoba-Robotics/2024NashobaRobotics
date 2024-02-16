package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class GrabberToShoot extends Command{
    LoaderSubsystem loader = RobotContainer.loader;

    public GrabberToShoot(){
        addRequirements(loader);
    }

    @Override
    public void execute() {
        loader.setRollerSpeed(Presets.Loader.SHOOT_SPEED);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
