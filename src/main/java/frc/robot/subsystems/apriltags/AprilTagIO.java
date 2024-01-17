package frc.robot.subsystems.apriltags;

import org.littletonrobotics.junction.AutoLog;

public interface AprilTagIO{
    @AutoLog
    public static class AprilTagIOInputs{
        public double x = 0;    //m
        public double y = 0;    //m
        public double z = 0;    //m

        public double timeStamp = 0;    //ms
        public boolean hasTarget = false;
    }
}
