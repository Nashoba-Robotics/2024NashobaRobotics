package frc.robot.subsystems.joystick;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class JoystickIOSwitchController implements JoystickIO{
    private CommandJoystick controller;

    public JoystickIOSwitchController(){
        controller = new CommandJoystick(2);
    }

    public void updateInputs(JoystickIOInputs inputs) {
        inputs.leftJoystickX = controller.getX();
        inputs.leftJoystickY = -controller.getY();

        inputs.rightJoystickX = controller.getZ();
        inputs.rightJoystickY = -controller.getTwist();
    }

    public CommandJoystick getLeftJoystick(){
        return null;
    }
    public CommandJoystick getRightJoystick(){
        return null;
    }
    public CommandJoystick getOperatorController(){
        return null;
    }
}
