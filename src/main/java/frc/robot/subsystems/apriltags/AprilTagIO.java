package frc.robot.subsystems.apriltags;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagIO{
    @AutoLog
    public static class AprilTagIOInputs{
        public Pose3d pos = new Pose3d();

        public double timeStamp = 0;    //ms
        public boolean hasTarget = false;
        public int tagsSeen = 0;
        public double ambiguity = 0;    //ratio for ambiguity

        public double yaw = 0;  //rad
    }

    public default void updateInputs(AprilTagIOInputs inputs) {}
}
