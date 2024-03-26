package frc.robot.subsystems.notedetection;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class NoteDetectorIOPhotonVision implements NoteDetectorIO{
    PhotonCamera camera;

    public NoteDetectorIOPhotonVision(){
        camera = new PhotonCamera(Constants.Cameras.NoteDetection.CAMERA_NAME);
    }

    @Override
    public void updateInputs(NoteDetectorIOInputs inputs) {
        PhotonPipelineResult r = camera.getLatestResult();

        PhotonTrackedTarget target = r.getBestTarget();
        inputs.area = target.getArea();
        inputs.yaw = target.getYaw();
        inputs.pitch = target.getPitch();


        // Pose3d robotPos = new Pose3d(RobotContainer.drive.getPose());
        Pose3d robotPos = new Pose3d();

        
        // List<PhotonTrackedTarget> targets = r.getTargets();
        // inputs.noteCount = targets.size();
        
        // int i = 0; 
        // for(PhotonTrackedTarget target : targets){
        //     Transform3d cameraToTarget = target.getBestCameraToTarget();
        //     Transform3d robotToTarget = Constants.Cameras.NoteDetection.ROBOT_TO_CAMERA.plus(cameraToTarget);
        //     if(i < inputs.notePos.length){
        //         inputs.notePos[i] = (robotPos.plus(robotToTarget).toPose2d());
        //         inputs.xs[i] = target.getPitch();
        //         inputs.ys[i] = target.getYaw();
        //     } 
        //     i++;
        // }
    }
}
