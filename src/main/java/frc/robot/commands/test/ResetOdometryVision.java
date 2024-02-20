package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetOdometryVision extends Command{
    
    private DriveSubsystem drive;

    public ResetOdometryVision(DriveSubsystem drive) {
        this.drive = drive;
    }
    
    @Override
    public void initialize() {
        drive.resetPose(AprilTagManager.getLeftRobotPos().toPose2d());
    }

}
