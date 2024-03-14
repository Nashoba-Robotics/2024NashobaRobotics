package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class LoaderToClimbPrep extends Command {
 
    private LoaderSubsystem loader = RobotContainer.loader;

    public LoaderToClimbPrep() {
        addRequirements(loader);
    }

    @Override
    public void initialize() {
        loader.setPivot(Presets.Loader.CLIMB_PREP_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Presets.Loader.CLIMB_PREP_POS.getRadians() - loader.getPivotAngle().getRadians()) < Presets.Loader.POS_TOLERANCE.getRadians();
    }

}
