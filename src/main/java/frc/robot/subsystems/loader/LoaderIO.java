package frc.robot.subsystems.loader;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;

public interface LoaderIO {
    //2 motors: 1 for pivot, 1 for rollers (falcons)
    /*
     * Wrist and loading
     */
    @AutoLog
    public static class LoaderIOInputs {
        public double pivotPosition = 0; //rad
        public double pivotVelocity = 0; //rad/s
        public double pivotVoltage = 0; //volts
        public double pivotStatorCurrent = 0; //amps
        public double pivotSupplyCurrent = 0; //amps

        public double rollerPosition = 0; //rad
        public double rollerVelocity = 0; //rad/s
        public double rollerVoltage = 0; //volts
        public double rollerStatorCurrent = 0; //amps
        public double rollerSupplyCurrent = 0; //amps

        public boolean loaderSensor = false;
        public boolean shooterSensor = false;
    }

    public default void updateInputs(LoaderIOInputs inputs) {}

    public abstract void setPivotPosition(Rotation2d position);
    public abstract void setPivotRotorPos(Rotation2d position);
    public abstract void setRollerSpeed(double speed);

    public abstract TalonFXConfiguration getPivotConfig();
    public abstract void setPivotConfig(TalonFXConfiguration config);

    public abstract void setLoaderkG(double kG);
    public abstract void setLoaderkS(double kS);
    public abstract void setLoaderkV(double kV);
    public abstract void setLoaderkP(double kP);
    public abstract void setLoaderkD(double kD);
    public abstract void setPivotSpeed(double speed);

    // public abstract boolean getLoaderSensor();
    // public abstract boolean getShooterSensor();
}
