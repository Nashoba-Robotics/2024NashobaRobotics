package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;

public class GyroIOPigeon2 implements GyroIO {
    
    private Pigeon2 gyro;
    private Pigeon2Configurator gyroConfigurator;
    private Pigeon2Configuration gyroConfig;

    public GyroIOPigeon2() {
        gyro = new Pigeon2(Constants.Misc.GYRO_PORT, "rio");
        gyroConfigurator = gyro.getConfigurator();
        config();

    }

    private void config() {
        gyroConfig = new Pigeon2Configuration();
        gyroConfig.MountPose.MountPoseYaw = -0.263672;
        gyroConfig.MountPose.MountPosePitch = 0.307617;
        gyroConfig.MountPose.MountPoseRoll = -0.483398;
        gyroConfigurator.apply(gyroConfig);
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = gyro.getYaw().getValue();
        inputs.pitch = gyro.getPitch().getValue();
        inputs.roll = gyro.getRoll().getValue();
    }

    public void setYaw(double angle) {
        gyro.setYaw(angle);
    }

}
