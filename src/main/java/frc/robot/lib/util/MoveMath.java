package frc.robot.lib.util;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    //https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/
    public static Translation3d getBallisticTrajectory(){
        //Use Fixed Speed with Moving Target:
        //  Treat the speaker as a moving target and the robot as a stationary one
        Pose2d robotPos = drive.getPose();
        double robotPosX = robotPos.getX();
        double robotPosY = robotPos.getY();
        double robotPosZ = 0.6;   //TODO: Find height of robot

        Translation3d targetPos = Constants.Field.getSpeakerPos();
        double targetPosX = targetPos.getX();
        double targetPosY = targetPos.getY();   //TODO: Check that x, y, z are where i think they are
        double targetPosZ = targetPos.getZ();

        ChassisSpeeds targetSpeeds = drive.getFieldRelativeSpeeds();
        double targetSpeedX = targetSpeeds.vxMetersPerSecond * (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -1 : 1);
        double targetSpeedY = targetSpeeds.vyMetersPerSecond * (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -1 : 1);
        double targetSpeedZ = 0;
        Logger.recordOutput("Field Relative Speeds", targetSpeeds);

        double G = 9.81;

        double A = robotPosX;
        double B = robotPosZ;
        double C = robotPosY;
        double M = targetPosX;
        double N = targetPosZ;
        double O = targetPosY;
        double P = targetSpeedX;
        double Q = targetSpeedZ;
        double R = targetSpeedY;
        double S = NOTE_SPEED;

        double H = M - A;
        double J = O - C;
        double K = N - B;
        double L = -.5f * G;

        // Quartic Coeffecients
        double c0 = G * G;
        double c1 = 0;
        double c2 = (P * P + R * R) + (K * -G) - S * S;
        double c3 = 2 * (H * P + K * R);
        double c4 = K * K + H * H + J * J;

        double[] times = Quartic.solveQuartic(c0, c1, c2, c3, c4);
        Logger.recordOutput("Possible times", times);
        double time = 0;

        for(double t : times){
            if(t > 0){
                time = t;
                break;
            }
        }
        
        Translation3d notePosVector = new Translation3d(
            ((H+P*time)),  //X
            ((J+R*time)),  //Y
            ((K+Q*time-L*time*time))   //Z (Up down)
        );


        return notePosVector;
    }

    public static double[] getShootWhileMoveBallistics2() {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getRobotRelativeSpeeds(), drive.getPose().getRotation());
        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        Logger.recordOutput("chassisspeeds", chassisSpeeds);
        Pose2d robotPos = drive.getPose();
        Translation3d speakerPose = Constants.Field.getSpeakerPos();

        // A lot of the code from this point forward is from here:
        // https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/
        double G = 9.81;
        double target_pos_x = speakerPose.getX();
        double target_pos_y = speakerPose.getZ();
        double target_pos_z = speakerPose.getY();
        double target_vel_x = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? chassisSpeeds.vxMetersPerSecond : -chassisSpeeds.vxMetersPerSecond;
        double target_vel_y = 0;
        double target_vel_z = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? chassisSpeeds.vyMetersPerSecond : -chassisSpeeds.vyMetersPerSecond;
        double proj_pos_x = robotPos.getX();
        double proj_pos_y = 0.6;
        double proj_pos_z = robotPos.getY();
        double proj_speed = NOTE_SPEED;

        double A = proj_pos_x;
        double B = proj_pos_y;
        double C = proj_pos_z;
        double M = target_pos_x;
        double N = target_pos_y;
        double O = target_pos_z;
        double P = target_vel_x;
        double Q = target_vel_y;
        double R = target_vel_z;
        double S = proj_speed;

        double H = M - A;
        double J = O - C;
        double K = N - B;
        double L = -.5f * G;

        // Quartic Coeffecients
        double c0 = G * G;
        double c1 = 0;
        double c2 = (P * P + R * R) + (K * -G) - S * S;
        double c3 = 2 * (H * P + K * R);
        double c4 = K * K + H * H + J * J;

        double[] q_sols = new double[5];
        q_sols = Quartic.solveQuartic(c0, c1, c2, c3, c4);
        double[] times = new double[4];
        for (int i = 0; i < times.length; i++) {
            times[i] = q_sols[i];
        }
        Logger.recordOutput("ShootOnTheFly/times", times);

        Arrays.sort(times);

        Translation3d[] solution_poses = new Translation3d[2];
        double[] new_dist = new double[2];
        solution_poses[0] = new Translation3d();
        solution_poses[1] = new Translation3d();
        int num_sols = 0;

        for (int i = 0; i < times.length; i++) {
            double t = times[i];

            if (t <= 0 || Double.isNaN(t)) {
                continue;
            }

            float x_note_speed = (float)((H + P * t) / t);
            float z_note_speed = (float) ((K + Q * t - L * t * t) / t);
            float y_note_speed = (float) ((J + R * t) / t);

            solution_poses[num_sols] = new Translation3d(
                    x_note_speed,   //Order doesn't really matter.
                    z_note_speed,
                    y_note_speed);
            


            Pose2d newPos = new Pose2d(robotPos.getX() + x_note_speed*t, robotPos.getY() + chassisSpeeds.vyMetersPerSecond*t, drive.getPose().getRotation());
            new_dist[num_sols] = 
                // newPos.getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                Math.hypot(x_note_speed*t, y_note_speed*t);
            num_sols++;
        }
        Translation3d sol_pose = solution_poses[0];
        Logger.recordOutput("ShootOnTheFly/pose", sol_pose);
        Rotation2d holo_align_angle = new Rotation2d(sol_pose.getX(), sol_pose.getZ());
        Logger.recordOutput("Shoot Move Dist", new_dist);

        double[] ret_val = new double[2];

        ret_val[0] = holo_align_angle.getRadians();
        // ret_val[1] = new Rotation2d(Math.hypot(sol_pose.getX(), sol_pose.getZ()), sol_pose.getY()).getDegrees() + ShooterConstants.encoderOffsetFromHorizontal - 7;
        ret_val[1] = new_dist[0];

        Logger.recordOutput("ShootOnTheFly/angle", Math.toDegrees(ret_val[0]));
        Logger.recordOutput("ShootOnTheFly/retval", ret_val);
        return ret_val;
    }
    
    public static Rotation2d getShooterSpeedFromDistance(double dist) {
        return Rotation2d.fromRadians(
            Math.min(
                Math.max(
                    Constants.Arm.SPEED_SLOPE * dist + Constants.Arm.SPEED_INTERCEPT, Constants.Arm.MIN_SPEED
                    ),
                Constants.Arm.MAX_SPEED
            )
            );
    }
}
