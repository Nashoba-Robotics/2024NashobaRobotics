package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

public class NoteToLoader extends Command{
    LoaderSubsystem loader = RobotContainer.loader;

    public NoteToLoader(){
        addRequirements(loader);
    }

    @Override
    public void execute() {
        double speed = !RobotContainer.sensors.getLoaderSensor() ? 0 : Presets.Loader.TO_LOADER_TRANSITION; 
        loader.setRollerSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        loader.setRollerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.sensors.getLoaderSensor();    //TODO: Add timer
    }
}
