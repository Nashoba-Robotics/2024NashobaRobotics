package frc.robot.commands.setters.units.loader;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class GrabberToIntake extends Command {
    
    private ArmSubsystem arm = RobotContainer.arm;

    public GrabberToIntake() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setLoaderSpeed(Presets.Loader.INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getLoaderPivotAngle().getRadians() - Presets.Loader.INTAKE_SPEED.getRadians()) < Presets.Loader.SPEED_TOLERANCE.getRadians();
    }

}
