package frc.robot.subsystems.apriltags;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;

public class AprilTagManagerTest implements AprilTagIO{
    PhotonCamera camera;
    PhotonPoseEstimator poseEstimator;
    
    double lastEstTimestamp = 0;
    boolean exists = true;

    public AprilTagManagerTest(){
        camera = new PhotonCamera("Yi's_Little_Buddy");//TODO: Figure out a fail-safe if "Photonvision doesn't exist"
        
        Transform3d kRobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(13.0), 0.0, 0.0), new Rotation3d(0, 18/180*Math.PI, 0));
        String filePath = Filesystem.getDeployDirectory().getPath() + "/TestPositions.json";

        try {
            poseEstimator = new PhotonPoseEstimator(
                new AprilTagFieldLayout(filePath),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                kRobotToCam);
            exists = true;
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + filePath, e.getStackTrace());
            exists = false;
        }
    }

     public void updateInputs(AprilTagIOInputs inputs) {
        PhotonPipelineResult r = camera.getLatestResult();
        inputs.hasTarget = r.hasTargets();
        inputs.timeStamp = r.getTimestampSeconds()/1000;

        Pose3d pos = poseEstimator.update().get().estimatedPose;
        inputs.x = pos.getX();
        inputs.y = pos.getY();
        inputs.z = pos.getZ();
     }
}
