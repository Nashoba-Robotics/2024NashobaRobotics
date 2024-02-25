package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class LoaderToIntake extends Command {
    private LoaderSubsystem loader = RobotContainer.loader;

    public LoaderToIntake() {
        addRequirements(loader);
    }

    @Override
    public void initialize() {
        loader.setPivot(Presets.Loader.INTAKE_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(loader.getPivotAngle().getRadians() - Presets.Loader.INTAKE_POS.getRadians()) < Presets.Loader.POS_TOLERANCE.getRadians();
    }

}
