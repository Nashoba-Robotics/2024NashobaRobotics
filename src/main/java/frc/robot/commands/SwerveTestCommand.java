package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Module;

public class SwerveTestCommand extends Command {

    private DriveSubsystem drive;

    private long lastTimeStamp;
    private double lastAngle;

    public SwerveTestCommand() {
       drive = RobotContainer.drive;
       addRequirements(drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("SetX", 0);
        SmartDashboard.putNumber("SetY", 0);
        SmartDashboard.putNumber("SetOmega", 0);
        lastTimeStamp = System.currentTimeMillis();
        lastAngle = drive.getGyroAngle().getRadians();

    }

    @Override
    public void execute() {
        double x = SmartDashboard.getNumber("SetX", 0);
        double y = SmartDashboard.getNumber("SetY", 0);
        double omega = SmartDashboard.getNumber("SetOmega", 0);

        SmartDashboard.putNumber("RadiansPerSecond", (drive.getGyroAngle().getRadians() - lastAngle) / ((System.currentTimeMillis() - lastTimeStamp)/1000.));
        lastAngle = drive.getGyroAngle().getRadians();
        lastTimeStamp = System.currentTimeMillis();

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);

        drive.set(speeds);

    }

    @Override
    public void end(boolean interrupted) {
        drive.set(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
