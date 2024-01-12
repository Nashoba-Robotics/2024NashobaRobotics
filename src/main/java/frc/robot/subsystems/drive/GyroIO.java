package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double yaw = 0; // Rot
        public double pitch = 0; // Rot
        public double roll = 0; // Rot
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void setYaw(double angle) {}
}
