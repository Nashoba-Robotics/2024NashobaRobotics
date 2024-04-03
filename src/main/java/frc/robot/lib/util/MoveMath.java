package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class MoveMath {
    public static DriveSubsystem drive = RobotContainer.drive;
    private static final double NOTE_SPEED = 17;    //m/s
    public static Translation3d aimToSpeaker(){
        Translation3d speakerPos = Constants.Field.getSpeakerPos();
        Pose2d drivePos = drive.getPose();
        ChassisSpeeds driveSpeeds = drive.getFieldRelativeSpeeds();
        Rotation2d robotToSpeakerAngle = Rotation2d.fromRadians(Math.atan2(
            speakerPos.getY() - drivePos.getY(),
            speakerPos.getX() - drivePos.getX())
        );

        double noteVelX = NOTE_SPEED*robotToSpeakerAngle.getCos();
        double robotVelX = driveSpeeds.vxMetersPerSecond;

        //Hotdog
        double t = Math.abs(noteVelX + robotVelX)/drivePos.getX();
        
        double noteVelY = NOTE_SPEED * robotToSpeakerAngle.getCos();
        double robotVelY = driveSpeeds.vyMetersPerSecond;

        double yOffset = (noteVelY + robotVelY) * t;
        double xOffset = (noteVelX + robotVelX) * t;

        Translation3d aimPos = new Translation3d(speakerPos.getX()-xOffset, speakerPos.getY()-yOffset, speakerPos.getZ());
        return aimPos;
    }
}
