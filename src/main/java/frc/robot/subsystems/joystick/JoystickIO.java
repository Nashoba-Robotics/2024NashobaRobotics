package frc.robot.subsystems.joystick;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.lib.util.JoystickValues;

public interface JoystickIO {
    
    @AutoLog
    public static class JoystickIOInputs {
        public double driveLeftJoystickX = 0;
        public double driveLeftJoystickY = 0;
        public double driveRightJoystickX = 0;
        public double driveRightJoystickY = 0;

        public double operatorJoystickLeftX = 0;
        public double operatorJoystickLeftY = 0;
        public double operatorJoystickRightX = 0;
        public double operatorJoystickRightY = 0;
    }

    public default void updateInputs(JoystickIOInputs inputs) {}

    public abstract JoystickValues getLeftJoystickValues();
    public abstract JoystickValues getRightJoystickValues();

    public abstract CommandJoystick getDriverController();
    public abstract CommandJoystick getOperatorController();

}
