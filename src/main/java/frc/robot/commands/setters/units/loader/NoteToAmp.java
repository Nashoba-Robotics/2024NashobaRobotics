package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class NoteToAmp extends Command{
    LoaderSubsystem loader = RobotContainer.loader;

    public NoteToAmp(){
        addRequirements(loader);
    }
    @Override
    public void execute() {
        loader.setRollerSpeed(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        loader.setRollerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return !loader.getLoaderSensor();
    }
}
