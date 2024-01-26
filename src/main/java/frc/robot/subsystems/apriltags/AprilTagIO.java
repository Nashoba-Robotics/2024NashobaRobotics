package frc.robot.subsystems.apriltags;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagIO{
    @AutoLog
    public static class AprilTagIOInputs{
        public Pose3d pos = new Pose3d();
        public double x = 0;    //m
        public double y = 0;    //m
        public double z = 0;    //m

        public double timeStamp = 0;    //ms
        public boolean hasTarget = false;
        public int tagsSeen = 0;
        public double ambiguity = -1;    //ratio for ambiguity

        public double yaw = 0;
    }

    public default void updateInputs(AprilTagIOInputs inputs) {}
}
