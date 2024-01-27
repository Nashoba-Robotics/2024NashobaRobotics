package frc.robot.subsystems.apriltags;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class AprilTagIOPhotonVision implements AprilTagIO{
    PhotonCamera camera1;
    PhotonPoseEstimator poseEstimator;
    AprilTagFieldLayout layout;
    boolean exists;

    public AprilTagIOPhotonVision(){
        camera1 = new PhotonCamera(Constants.AprilTags.CAMERA1_NAME);
        
        try{
            layout = new AprilTagFieldLayout(Constants.AprilTags.LAYOUT_PATH);
            poseEstimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera1,
                Constants.AprilTags.ROBOT_TO_CAMERA1);
            exists = true;

            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);   //TODO: YAAAY

        } catch(IOException e){
            DriverStation.reportError("Unable to open trajectory: " + Constants.AprilTags.LAYOUT_PATH, e.getStackTrace());
            exists = false; 
        }
    }

    public void updateInputs(AprilTagIOInputs inputs){
        PhotonPipelineResult r = camera1.getLatestResult();
        inputs.hasTarget = r.hasTargets();

        if(inputs.hasTarget) inputs.yaw = r.getBestTarget().getYaw() * Constants.TAU/360;
        else inputs.yaw = 0;

        List<PhotonTrackedTarget> targets = r.getTargets();
        inputs.tagsSeen = targets.size();


        if(inputs.tagsSeen == 1){
            inputs.ambiguity = targets.get(0).getPoseAmbiguity();   //<-- TODO: More testing with this. May be giving periodic zeroes
        }
        else{
            inputs.ambiguity = 0;
        }

        Optional<EstimatedRobotPose> estimator = poseEstimator.update();
        
        if(!exists){
            inputs.pos = null;
        }
        else if(estimator.isPresent()){
            inputs.pos = estimator.get().estimatedPose;

            inputs.timeStamp = r.getTimestampSeconds();
        }
    }

    public void switchPipeline(){

    }
    
}
