package frc.robot.subsystems.apriltags;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class AprilTagIOPhotonVision implements AprilTagIO{
    PhotonCamera camera1;
    PhotonPoseEstimator poseEstimator;
    boolean exists;

    double lastX = 0;
    double lastY = 0;
    double lastZ = 0;


    public AprilTagIOPhotonVision(){
        camera1 = new PhotonCamera(Constants.AprilTags.CAMERA1_NAME);

        try{
            poseEstimator = new PhotonPoseEstimator(
                new AprilTagFieldLayout(Constants.AprilTags.LAYOUT_PATH),
                // AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                // PoseStrategy.LOWEST_AMBIGUITY,
                camera1,
                Constants.AprilTags.ROBOT_TO_CAMERA1);
            exists = true;
        } catch(IOException e){
            DriverStation.reportError("Unable to open trajectory: " + Constants.AprilTags.LAYOUT_PATH, e.getStackTrace());
            exists = false; 
        }
    }

    public void updateInputs(AprilTagIOInputs inputs){
        PhotonPipelineResult r = camera1.getLatestResult();
        inputs.hasTarget = r.hasTargets();
        inputs.timeStamp = r.getTimestampSeconds();


        Optional<EstimatedRobotPose> estimator = poseEstimator.update();
        if(!exists){
            inputs.x = 17.68;
            inputs.y = 17.68;
            inputs.z = 17.68;
        }
        else if(estimator.isPresent()){
            inputs.pos = estimator.get().estimatedPose;
            inputs.x = inputs.pos.getX();
            inputs.y = inputs.pos.getY();
            inputs.z = inputs.pos.getZ();

            lastX = inputs.pos.getX();
            lastY = inputs.pos.getY();
            lastZ = inputs.pos.getZ();
        }
        else{
            inputs.x = lastX;
            inputs.y = lastY;
            inputs.z = lastZ;
        }
        
    }

    public void switchPipeline(){

    }
    
}
