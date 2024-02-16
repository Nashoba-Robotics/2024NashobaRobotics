package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class NoteToShooter extends Command{
    LoaderSubsystem loader = RobotContainer.loader;
    Timer timer;

    public NoteToShooter(){
        timer = new Timer();
        addRequirements(loader);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double speed = loader.getShooterSensor() ? 0 : Presets.Loader.TO_SHOOTER_TRANSITION;
        loader.setRollerSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        loader.setRollerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return loader.getShooterSensor() || timer.get() > 0.5;   //TODO: Check initially if we actually have a note
    }
}
