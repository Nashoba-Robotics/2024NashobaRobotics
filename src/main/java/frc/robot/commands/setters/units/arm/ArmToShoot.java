package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ArmToShoot extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    DriveSubsystem drive = RobotContainer.drive;

    double angle;

    public ArmToShoot(){
        angle = 0;

        addRequirements(arm);
    }
    @Override
    public void execute() {
        double y = Constants.Field.getSpeakerPos().getZ()-Constants.Robot.SHOOTER_HEIGHT;
        // Translation2d shootPos = new Translation2d(drive.getPose().getX(), y)
        double dist = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
        dist -= 0.22;
        angle = -Math.atan2(y, dist);

       // TODO: Check if the angle is within our domain. 
        arm.setArmPivot(Rotation2d.fromRadians(angle));
    }
    @Override
    public boolean isFinished() {
        return Math.abs(arm.getArmPivotAngle().getRadians()-angle) <= Presets.Arm.POS_TOLERANCE.getRadians();
    }
}
