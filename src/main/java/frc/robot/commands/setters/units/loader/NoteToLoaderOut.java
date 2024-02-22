package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.NoteState;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class NoteToLoaderOut extends Command {
    
    private LoaderSubsystem loader = RobotContainer.loader;

    public NoteToLoaderOut() {
        addRequirements(loader);
    }

    public void execute() {
        loader.setRollerSpeed(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        loader.setRollerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return !loader.getLoaderSensor() || loader.getShooterSensor();
    }

}
