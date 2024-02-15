package frc.robot.commands.setters.units;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class StopAllRollers extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    IntakeSubsystem intake = RobotContainer.intake;
    
    public StopAllRollers(){
        addRequirements(arm, intake);
    }

    @Override
    public void execute() {
        arm.setShooterSpeed(Rotation2d.fromDegrees(0));
        arm.setLoaderSpeed(Rotation2d.fromRotations(0));
        intake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return arm.getLoaderSpeed().getRotations() == 0 && arm.getShooterSpeed().getRotations() == 0 && false;
    }
}
