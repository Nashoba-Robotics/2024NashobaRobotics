package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class LoaderToSource extends Command{
    LoaderSubsystem loader = RobotContainer.loader;

    public LoaderToSource(){
        addRequirements(loader);
    }

    @Override
    public void execute() {
        if(!loader.getLoaderSensor() && !loader.getShooterSensor()) loader.setPivot(Presets.Loader.SOURCE_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(loader.getPivotAngle().getRadians() - Presets.Loader.SOURCE_POS.getRadians()) <= Presets.Loader.POS_TOLERANCE.getRadians()
        || loader.getLoaderSensor() || loader.getShooterSensor();
    }
}
