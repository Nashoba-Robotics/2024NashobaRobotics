package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class LoaderToAmpPosition extends Command {
    
    private ArmSubsystem arm = RobotContainer.arm;

    public LoaderToAmpPosition() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setLoaderPivot(Presets.Loader.AMP_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getLoaderPivotAngle().getRadians()) < Presets.Loader.POS_TOLERANCE.getRadians();
    }

}
