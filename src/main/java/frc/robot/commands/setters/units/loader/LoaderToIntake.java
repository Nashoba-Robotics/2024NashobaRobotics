package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class LoaderToIntake extends Command {
    
    private ArmSubsystem armSubsystem = RobotContainer.arm;

    public LoaderToIntake() {
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setLoaderPivot(Presets.Loader.INTAKE_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armSubsystem.getLoaderPivotAngle().getRadians() - Presets.Loader.INTAKE_POS.getRadians()) < Presets.Loader.POS_TOLERANCE.getRadians();
    }

}
