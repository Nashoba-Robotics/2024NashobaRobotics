package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AimToSpeakerCommand extends Command{
    DriveSubsystem drive = RobotContainer.drive;
    ChassisSpeeds chassisSpeeds;
    PIDController angleController;

    public AimToSpeakerCommand(){
        chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        angleController = new PIDController(0.5, 0, 0);
        angleController.setTolerance(Constants.TAU/360);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        

        double goal = Math.atan2(
            drive.getPose().getY() - Constants.Field.getSpeakerPos().getY(),
            drive.getPose().getX() - Constants.Field.getSpeakerPos().getX()
            );
        double currAngle = drive.getYaw().getRadians();
        double angleDiff = goal - drive.getGyroAngle().getRadians();

        goal = angleDiff < Constants.TAU/2 ?
            currAngle + angleDiff :
            currAngle - (Constants.TAU - angleDiff);

        chassisSpeeds.omegaRadiansPerSecond = angleController.calculate(currAngle, goal);

        Logger.recordOutput("Speaker Angle", goal);

        drive.set(chassisSpeeds);
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint();
    }
}
