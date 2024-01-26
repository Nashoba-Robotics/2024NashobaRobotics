package frc.robot.subsystems.apriltags;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class AprilTagIOPhotonVision implements AprilTagIO{
    PhotonCamera camera1;
    PhotonPoseEstimator poseEstimator;
    AprilTagFieldLayout layout;
    boolean exists;

    double lastX = 0;
    double lastY = 0;
    double lastZ = 0;

    double lastTimeStamp = 0;


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

        if(inputs.hasTarget) inputs.yaw = r.getBestTarget().getYaw();
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

            inputs.timeStamp = r.getTimestampSeconds();
            lastTimeStamp = inputs.timeStamp;
        }
        else{
            inputs.x = lastX;
            inputs.y = lastY;
            inputs.z = lastZ;

            inputs.timeStamp = lastTimeStamp;
        }
        
    }

    public void switchPipeline(){

    }
    
}
