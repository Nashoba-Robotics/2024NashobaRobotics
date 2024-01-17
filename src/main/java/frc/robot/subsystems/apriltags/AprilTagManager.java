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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagManager extends SubsystemBase{
    // PhotonCamera cam = new PhotonCamera("Yi's Little Buddy");
    // //Is this correct?
    // Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(13), 0.0, 0.0), new Rotation3d(0,0,0));

    //  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update();
    // }
    PhotonCamera camera;
    PhotonPoseEstimator poseEstimator;
    
    double lastEstTimestamp = 0;
    boolean exists = true;

    public AprilTagManager(){
        camera = new PhotonCamera("Yi's_Little_Buddy");//TODO: Figure out a fail-safe if "Photonvision doesn't exist"
        // PhotonPipelineResult x = camera.getLatestResult();
         Transform3d kRobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(13.0), 0.0, 0.0), new Rotation3d(0, 18/180*Math.PI, 0));

        try {
            poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
            exists = true;
        } catch (IOException e) {
            exists = false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Exists", exists);
    }

    private static AprilTagManager instance;
    public static AprilTagManager getInstance() throws IOException{ 
        if(instance == null) instance = new AprilTagManager();
        return instance;
    }

    public boolean hasTargets(){
        return getLatestResult().hasTargets();
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    // public Optional<EstimatedRobotPose> getEstimatePose(){
    //     return poseEstimator.update();
    // }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        // if(exists) 
        return poseEstimator.update();

        // double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        // boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        // if (newResult) lastEstTimestamp = latestTimestamp;

        // return Optional.empty();

    }
}
