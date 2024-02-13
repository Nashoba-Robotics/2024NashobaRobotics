package frc.robot.subsystems.arm;

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
    }

    public default void updateInputs(LoaderIOInputs inputs) {}
    public void setPivotPosition(Rotation2d position);
    public void setPivotRotorPos(Rotation2d position);
    public void setRollerSpeed(Rotation2d speed);

    public TalonFXConfiguration getPivotConfig();
    public void setPivotConfig(TalonFXConfiguration config);

    public void setLoaderkG(double kG);
    public void setLoaderkS(double kS);
    public void setLoaderkV(double kV);
    public void setLoaderkP(double kP);
    public void setLoaderkD(double kD);
    public void setPivotSpeed(double speed);
}
