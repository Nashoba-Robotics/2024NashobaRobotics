package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    /*
     * Pivoting arm and shooter
     */

     @AutoLog
     public static class ArmIOInputs{
        public double armAbsolutePosition = 0; //rad
        public double armRotorPosition = 0;   //rad
        
        public double armStatorCurrent = 0; //Amps
        public double armSupplyCurrent = 0; //Amps
        public double armVoltage = 0;   //Volts

        public double shooterSpeed = 0; //rps
        public double shooterStatorCurrent = 0;    //Amps
        public double shooterSupplyCurrent = 0;
        public double shooterVoltage = 0;   //Volts
     }

     public abstract void setAngle(double angle);
     public abstract void setShooterSpeed(double speed);
}
