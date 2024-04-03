package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SetOdometryCommand extends Command{
    DriveSubsystem drive;

    public SetOdometryCommand(DriveSubsystem drive){
        this.drive = drive;
    }
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Drive Angle", 0);
        SmartDashboard.putNumber("Drive X", 0);
        SmartDashboard.putNumber("Drive Y", 0);

        drive.overrideVisionOdo = true;
    }

    @Override
    public void execute() {
        double angle = SmartDashboard.getNumber("Drive Angle", 0);
        double x = SmartDashboard.getNumber("Drive X", 0);
        double y = SmartDashboard.getNumber("Drive Y", 0);
        drive.resetOdometryManualAngle(new Pose2d(x, y, Rotation2d.fromRadians(angle)), Rotation2d.fromRadians(angle));
    }

    @Override
    public void end(boolean interrupted) {
        drive.overrideVisionOdo = false;
    }
}
