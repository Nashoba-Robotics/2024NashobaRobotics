package frc.robot.subsystems.apriltags;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagIO{
    @AutoLog
    public static class AprilTagIOInputs {
        //left = ben, right = Yi
        public Pose3d leftPos = new Pose3d();
        public Pose2d leftPose2d = new Pose2d();

        public double leftTimeStamp = 0;    //ms 
        public boolean leftHasTarget = false;
        public int leftTagsSeen = 0;
        public double leftAmbiguity = 0;    //ratio for ambiguity

        public double leftYaw = 0;  //rad

        public Pose3d rightPos = new Pose3d();
        public Pose2d rightPose2d = new Pose2d();

        public double rightTimeStamp = 0;    //ms 
        public boolean rightHasTarget = false;
        public int rightTagsSeen = 0;
        public double rightAmbiguity = 0;    //ratio for ambiguity

        public double rightYaw = 0;  //rad

        double leftDistToSpeaker = 0;
        double rightDistToSpeaker = 0;
    }

    public default void updateInputs(AprilTagIOInputs inputs) {}
}
