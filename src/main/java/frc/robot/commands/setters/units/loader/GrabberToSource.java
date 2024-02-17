package frc.robot.commands.setters.units.loader;

import javax.print.attribute.standard.PresentationDirection;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class GrabberToSource extends Command{
    LoaderSubsystem loader = RobotContainer.loader;

    public GrabberToSource(){
        addRequirements(loader);
    }

    @Override
    public void execute() {
        double speed = loader.getLoaderSensor() ? 0 : Presets.Loader.SOURCE_SPEED;
        loader.setRollerSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return loader.getLoaderSensor();
    }
}
