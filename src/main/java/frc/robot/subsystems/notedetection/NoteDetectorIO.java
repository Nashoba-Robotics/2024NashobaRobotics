package frc.robot.subsystems.notedetection;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface NoteDetectorIO {
    @AutoLog
    public static class NoteDetectorIOInputs{
        public int noteCount = 0;
        // public double[] xs = new double[5];
        // public double[] ys = new double[5];
        public double pitch = 0;
        public double yaw = 0;
        public double area = 0;

        public boolean p3 = false;
        public boolean p4 = false;
        public boolean p5 = false;
        public boolean p6 = false;
        public boolean p7 = false;

        public Pose2d[] notePos = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};

    }

    public default void updateInputs(NoteDetectorIOInputs inputs){}
}
