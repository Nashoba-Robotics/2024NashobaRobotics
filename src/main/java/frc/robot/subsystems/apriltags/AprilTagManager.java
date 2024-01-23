package frc.robot.subsystems.apriltags;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Files;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.apriltags.AprilTagIO.AprilTagIOInputs;

public class AprilTagManager extends SubsystemBase {
    private AprilTagIO io;
    private static AprilTagIOInputsAutoLogged inputs = new AprilTagIOInputsAutoLogged();

    public AprilTagManager(){
        io = new AprilTagIOPhotonVision();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Camera", inputs);
        Logger.recordOutput("RobotPos (Camera)", inputs.pos.toPose2d());
        // List<PhotonTrackedTarget> targets = inputs.targets;
        // for(PhotonTrackedTarget target : targets){
        //     Logger.recordOutput(target.getFiducialId() + ": Tag Ambiguity", target.getPoseAmbiguity());
        // }

        // RobotContainer.drive.updateOdometryWithVision(inputs.pos.toPose2d(), inputs.timeStamp);

    }

    public static boolean hasTarget(){
        return inputs.hasTarget;
    }
    public static double getTimestamp(){
        return inputs.timeStamp;
    }

    public static Pose3d getRobotPos(){
        return inputs.pos;
    }

    public static double getRobotX(){
        return inputs.x;
    }
    public static double getRobotY(){
        return inputs.y;
    }
    public static double getRobotZ(){
        return inputs.z;
    }
}
