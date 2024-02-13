package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    /*
     * Pivoting arm and shooter
     */

     @AutoLog
     public static class ArmIOInputs{
        public double pivotAbsolutePosition = 0; //rad
        public double pivotRotorPosition = 0;   //rad
        public double pivotSpeed = 0; //rad/s
        
        public double pivotStatorCurrent = 0; //amps
        public double pivotSupplyCurrent = 0; //amps
        public double pivotVoltage = 0;   //volts

        public double shooterPosition = 0; //rad
        public double shooterSpeed = 0; //rad/s
        public double shooterStatorCurrent = 0;    //amps
        public double shooterSupplyCurrent = 0;
        public double shooterVoltage = 0;   //volts

        public boolean loaderSensor = false;
        public boolean shooterSensor = false;
     }

     public default void updateInputs(ArmIOInputs inputs) {}
     public abstract void setAngle(Rotation2d angle);
     public abstract void setPivotSpeed(double speed);
     public abstract void setPivotRotorPos(Rotation2d pos);
     public abstract void setShooterSpeed(Rotation2d speed);
     public abstract boolean getShooterSensor();
     public abstract boolean getLoaderSensor();

     public abstract void setPivotkG(double kG);
     public abstract void setPivotkS(double kS);
     public abstract void setPivotkV(double kV);
     public abstract void setPivotkP(double kP);
     public abstract void setPivotkD(double kD);
}
