package frc.robot.subsystems.notedetection;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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


        // Pose3d robotPos = new Pose3d(RobotContainer.drive.getPose());
        Pose3d robotPos = new Pose3d();

        PhotonTrackedTarget bestTarget = r.getBestTarget();
        inputs.yaw = bestTarget.getYaw();
        inputs.pitch = bestTarget.getPitch();

        double cameraToNoteY = (64.8794*Math.tan(0.0945988*inputs.pitch) + 65.6394)/100;
        double cameraToNoteX = Math.tan(1.68617*inputs.yaw-0.432508) * cameraToNoteY;

        Transform2d cameraToNote = new Transform2d(
            cameraToNoteX,
            cameraToNoteY,
            Rotation2d.fromRadians(Math.atan2(cameraToNoteX, cameraToNoteY)));  //Does this work?
        Transform2d robotToNote = Constants.Cameras.NoteDetection.ROBOT_TO_CAMERA.plus(cameraToNote);

        Pose2d notePos = robotPos.toPose2d().plus(robotToNote);

        
        List<PhotonTrackedTarget> targets = r.getTargets();
        inputs.noteCount = targets.size();
        
        int i = 0; 
        for(PhotonTrackedTarget target : targets){
            double pitch = target.getPitch();
            double y = 64.8794 * Math.tan(0.0945988*pitch) + 65.6394;

            double yaw = target.getYaw();
            double theta = 1.53532*yaw + 1.83981;

            double x = y/Math.tan(theta);

            Pose2d notePose = new Pose2d(robotPos.getX(), yaw, null);

            i++;
        }
    }
}
