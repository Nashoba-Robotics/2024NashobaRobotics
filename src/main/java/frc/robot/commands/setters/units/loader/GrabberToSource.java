package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

public class GrabberToSource extends Command{
    LoaderSubsystem loader = RobotContainer.loader;

    public GrabberToSource(){
        addRequirements(loader);
    }

    @Override
    public void initialize() {
        double speed = RobotContainer.sensors.getLoaderSensor() ? 0 : Presets.Loader.SOURCE_SPEED;
        loader.setRollerSpeed(speed);
    }

    @Override
    public void execute() {
        double speed = RobotContainer.sensors.getLoaderSensor() ? 0 : Presets.Loader.SOURCE_SPEED;
        loader.setRollerSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        // return loader.getLoaderSensor();
        return true;
    }
}
