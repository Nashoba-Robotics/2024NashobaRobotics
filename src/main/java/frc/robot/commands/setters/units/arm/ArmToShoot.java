package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ArmToShoot extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    DriveSubsystem drive = RobotContainer.drive;

    public ArmToShoot(){
        addRequirements(arm);
    }
    @Override
    public void execute() {
        double y = Constants.Field.SPEAKER_POSITION.getY()-Constants.Robot.SHOOTER_HEIGHT;
        double dist = drive.getPose().getTranslation().getDistance(Constants.Field.SPEAKER_POSITION);
        double angle = -Math.atan2(y, dist);

       // TODO: Check if the angle is within our domain. 
        arm.setArmPivot(Rotation2d.fromRadians(angle));
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
