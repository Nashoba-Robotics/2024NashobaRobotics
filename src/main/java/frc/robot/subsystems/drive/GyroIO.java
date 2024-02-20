package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double yaw = 0; // Rad
        public double constrainedYaw = 0;
        public double pitch = 0; // Rad
        public double roll = 0; // Rad

        public double xVelocity = 0; // Rad/s
        public double yVelocity = 0; // Rad/s
        public double zVelocity = 0; // Rad/s

    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void setYaw(double angle) {}
}
