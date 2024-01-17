package frc.robot.subsystsms.joystick;

public class JoystickIOSwitchController implements JoystickIO{
    private CommandJoystick controller;

    public JoystickkIOSwitchController(){
        controller = new CommandJoystick(2);
    }

    public void updateInputs(JoystickIOInputs inputs) {
        inputs.leftJoystickX = controller.getX();
        inputs.leftJoystickY = -controller.getY();

        inputs.rightJoystickX = controller.getZ();
        inputs.rightJoystickY = -controller.getTwist();
    }
}
