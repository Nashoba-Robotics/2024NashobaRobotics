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
    PhotonCamera camera2;
    PhotonPoseEstimator poseEstimator1;
    PhotonPoseEstimator poseEstimator2;
    AprilTagFieldLayout layout;
    boolean exists;

    public AprilTagIOPhotonVision(){
        camera1 = new PhotonCamera(Constants.AprilTags.CAMERA1_NAME);
        camera2 = new PhotonCamera(Constants.AprilTags.CAMERA2_NAME);
        
        try{
            layout = new AprilTagFieldLayout(Constants.AprilTags.LAYOUT_PATH);
            poseEstimator1 = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera1,
                Constants.AprilTags.ROBOT_TO_CAMERA1);

            poseEstimator2 = new PhotonPoseEstimator(
                layout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera2,
                Constants.AprilTags.ROBOT_TO_CAMERA2);
            exists = true;

            poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);   //TODO: YAAAY
            poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        } catch(IOException e){
            DriverStation.reportError("Unable to open trajectory: " + Constants.AprilTags.LAYOUT_PATH, e.getStackTrace());
            exists = false; 
        }
    }

    public void updateInputs(AprilTagIOInputs inputs){
        PhotonPipelineResult r = camera1.getLatestResult();
        inputs.leftHasTarget = r.hasTargets();

        if(inputs.leftHasTarget) inputs.leftYaw = r.getBestTarget().getYaw() * Constants.TAU/360;
        else inputs.leftYaw = 0;

        List<PhotonTrackedTarget> targets = r.getTargets();
        inputs.leftTagsSeen = targets.size();


        if(inputs.leftTagsSeen == 1){
            inputs.leftAmbiguity = targets.get(0).getPoseAmbiguity();   //<-- TODO: More testing with this. May be giving periodic zeroes
        }
        else{
            inputs.leftAmbiguity = 0;
        }

        Optional<EstimatedRobotPose> estimator = poseEstimator1.update();
        
        if(!exists){
            inputs.leftPos = null;
        }
        else if(estimator.isPresent()){
            inputs.leftPos = estimator.get().estimatedPose;

            inputs.leftTimeStamp = r.getTimestampSeconds();
        }

        r = camera2.getLatestResult();
        inputs.rightHasTarget = r.hasTargets();

        if(inputs.rightHasTarget) inputs.rightYaw = r.getBestTarget().getYaw() * Constants.TAU/360;
        else inputs.rightYaw = 0;

        targets = r.getTargets();
        inputs.rightTagsSeen = targets.size();


        if(inputs.rightTagsSeen == 1){
            inputs.rightAmbiguity = targets.get(0).getPoseAmbiguity();   //<-- TODO: More testing with this. May be giving periodic zeroes
        }
        else{
            inputs.rightAmbiguity = 0;
        }

        estimator = poseEstimator2.update();
        
        if(!exists){
            inputs.rightPos = null;
        }
        else if(estimator.isPresent()){
            inputs.rightPos = estimator.get().estimatedPose;

            inputs.rightTimeStamp = r.getTimestampSeconds();
        }

        
    }

    public void switchPipeline(){

    }
    
}
