package frc.robot.subsystems.apriltags;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class AprilTagIOPhotonVision implements AprilTagIO{
    PhotonCamera frontLeftCamera;
    PhotonCamera frontRightCamera;
    PhotonCamera backLeftCamera;
    PhotonCamera backRightCamera;
    PhotonPoseEstimator frontLeftPoseEstimator;
    PhotonPoseEstimator frontRightPoseEstimator;
    PhotonPoseEstimator backLeftPoseEstimator;
    PhotonPoseEstimator backRightPoseEstimator;
    AprilTagFieldLayout layout;
    boolean exists;

    public AprilTagIOPhotonVision(){
        frontRightCamera = new PhotonCamera(Constants.AprilTags.FRONT_RIGHT_CAMERA_NAME);   //Right
        frontLeftCamera = new PhotonCamera(Constants.AprilTags.FRONT_LEFT_CAMERA_NAME);   //Left
        // backLeftCamera = new PhotonCamera(Constants.AprilTags.BACK_LEFT_CAMERA_NAME);
        // backRightCamera = new PhotonCamera(Constants.AprilTags.BACK_RIGHT_CAMERA_NAME);
        
        try{
            layout = new AprilTagFieldLayout(Constants.AprilTags.LAYOUT_PATH);
            frontLeftPoseEstimator = new PhotonPoseEstimator(
                layout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                frontLeftCamera,
                Constants.AprilTags.ROBOT_TO_CAMERA_FRONT_LEFT);

            frontRightPoseEstimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                frontRightCamera,
                Constants.AprilTags.ROBOT_TO_CAMERA_FRONT_RIGHT);
            
            // backLeftPoseEstimator = new PhotonPoseEstimator(
            //     layout,
            //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            //     backLeftCamera,
            //     Constants.AprilTags.ROBOT_TO_CAMERA_BACK_LEFT);
            
            // backRightPoseEstimator = new PhotonPoseEstimator(
            //     layout, 
            //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            //     backRightCamera,
            //     Constants.AprilTags.ROBOT_TO_CAMERA_BACK_RIGHT);
            
            exists = true;

            frontLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            frontRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            // backLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            // backRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        } catch(IOException e){
            DriverStation.reportError("Unable to open trajectory: " + Constants.AprilTags.LAYOUT_PATH, e.getStackTrace());
            exists = false; 
        }
    }

    public void updateInputs(AprilTagIOInputs inputs){
        PhotonPipelineResult r = frontLeftCamera.getLatestResult();
        inputs.frontLeftHasTarget = r.hasTargets();

        if(inputs.frontLeftHasTarget) inputs.leftYaw = r.getBestTarget().getYaw() * Constants.TAU/360;
        else inputs.leftYaw = 0;

        List<PhotonTrackedTarget> targets = r.getTargets();
        inputs.leftTagsSeen = targets.size();

        int speakerID = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 7 : 4;
        for(PhotonTrackedTarget t : targets){
            if(t.getFiducialId() == speakerID){
                inputs.leftDistToSpeaker = t.getBestCameraToTarget().getX()*t.getBestCameraToTarget().getX() + t.getBestCameraToTarget().getY()*t.getBestCameraToTarget().getY();
            }
        }


        if(inputs.leftTagsSeen == 1){
            inputs.leftAmbiguity = targets.get(0).getPoseAmbiguity();   //<-- TODO: More testing with this. May be giving periodic zeroes
        }
        else{
            inputs.leftAmbiguity = 0;
        }

        Optional<EstimatedRobotPose> estimator = frontLeftPoseEstimator.update();
        
        if(!exists){
            inputs.frontLeftPos = null;
        }
        else if(estimator.isPresent()){
            inputs.frontLeftPos = estimator.get().estimatedPose;
            inputs.frontLeftPose2d = inputs.frontLeftPos.toPose2d();

            inputs.frontLeftTimeStamp = r.getTimestampSeconds();
        }


        r = frontRightCamera.getLatestResult();
        inputs.rightHasTarget = r.hasTargets();

        if(inputs.rightHasTarget) inputs.rightYaw = r.getBestTarget().getYaw() * Constants.TAU/360;
        else inputs.rightYaw = 0;

        targets = r.getTargets();
        inputs.rightTagsSeen = targets.size();

        for(PhotonTrackedTarget t : targets){
            if(t.getFiducialId() == speakerID){
                inputs.rightDistToSpeaker = t.getBestCameraToTarget().getX()*t.getBestCameraToTarget().getX() + t.getBestCameraToTarget().getY()*t.getBestCameraToTarget().getY();
            }
        }


        if(inputs.rightTagsSeen == 1){
            inputs.rightAmbiguity = targets.get(0).getPoseAmbiguity();   //<-- TODO: More testing with this. May be giving periodic zeroes
        }
        else{
            inputs.rightAmbiguity = 0;
        }

        estimator = frontRightPoseEstimator.update();
        
        if(!exists){
            inputs.rightPos = null;
        }
        else if(estimator.isPresent()){
            inputs.rightPos = estimator.get().estimatedPose;
            inputs.rightPose2d = inputs.rightPos.toPose2d();

            inputs.rightTimeStamp = r.getTimestampSeconds();
        }

        
    }

    public void switchPipeline(){

    }
    
}
