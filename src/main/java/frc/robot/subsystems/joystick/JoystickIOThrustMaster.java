package frc.robot.subsystems.joystick;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;

public class JoystickIOThrustMaster implements JoystickIO {
    
    private CommandJoystick leftJoystick;
    private CommandJoystick rightJoystick;
    private CommandJoystick operatorController;

    public JoystickIOThrustMaster() {
        leftJoystick = new CommandJoystick(Constants.Joystick.LEFT_JOYSTICK_PORT);
        rightJoystick = new CommandJoystick(Constants.Joystick.RIGHT_JOYSTICK_PORT);
        operatorController = new CommandJoystick(Constants.Joystick.OPERATOR_PORT);
    }

    public CommandJoystick getRightJoystick() {
        return rightJoystick;
    }

    public CommandJoystick getLeftJoystick() {
        return leftJoystick;
    }

    public CommandJoystick getOperatorController() {
        return operatorController;
    }

    public void updateInputs(JoystickIOInputs inputs) {
        inputs.leftJoystickX = leftJoystick.getX();
        inputs.leftJoystickY = -leftJoystick.getY();

        inputs.rightJoystickX = rightJoystick.getX();
        inputs.rightJoystickY = -rightJoystick.getY();
    }
    
}
