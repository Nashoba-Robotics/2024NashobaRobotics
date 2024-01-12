package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    
    @AutoLog
    public static class ModuleIOInputs {
        public double movePosition = 0; // m
        public double moveVelocity = 0; // m/s
        public double moveVoltage = 0; // Volts
        public double moveStatorCurrent = 0; // Amps
        public double moveSupplyCurrent = 0; // Amps

        public double turnAbsolutePosition = 0; // rot
        public double turnRotorPosition = 0; // rot
        public double turnVelocity = 0; // rot/s
        public double turnVoltage = 0; // Volts
        public double turnStatorCurrent = 0; // Amps
        public double turnSupplyCurrent = 0; // Amps
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void set(SwerveModuleState state) {}
    public SwerveModulePosition getPosition();
    public SwerveModuleState getState();

}