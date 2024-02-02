package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetOdometryCommand extends Command {

    private DriveSubsystem drive;

    public ResetOdometryCommand(DriveSubsystem drive) {
        this.drive = drive;
    }
    
    @Override
    public void initialize() {
        drive.resetPose(new Pose2d(8, 6, Rotation2d.fromDegrees(0)));
    }

}
