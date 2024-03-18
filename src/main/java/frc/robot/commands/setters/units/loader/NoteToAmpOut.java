package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

public class NoteToAmpOut extends Command{
    LoaderSubsystem loader = RobotContainer.loader;

    public NoteToAmpOut(){
        addRequirements(loader);
    }
    @Override
    public void execute() {
        loader.setRollerSpeed(0.25);
    }

    @Override
    public void end(boolean interrupted) {
        loader.setRollerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return (Governor.getDesiredRobotState() == RobotState.AMP && Governor.getLastRobotState() == RobotState.AMP_ADJ) || (!RobotContainer.sensors.getLoaderSensor() && !RobotContainer.sensors.getShooterSensor());
    }
}
