package frc.robot.subsystems.joystick;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public interface JoystickIO {
    
    @AutoLog
    public static class JoystickIOInputs {
        public double leftJoystickX = 0;
        public double leftJoystickY = 0;

        public double rightJoystickX = 0;
        public double rightJoystickY = 0;

        public double operatorJoystickLeftX = 0;
        public double operatorJoystickLeftY = 0;
        public double operatorJoystickRightX = 0;
        public double operatorJoystickRightY = 0;
    }

    public default void updateInputs(JoystickIOInputs inputs) {}

    public CommandJoystick getLeftJoystick();
    public CommandJoystick getRightJoystick();
    public CommandJoystick getOperatorController();

}
