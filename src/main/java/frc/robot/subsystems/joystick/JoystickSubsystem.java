package frc.robot.subsystems.joystick;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.lib.util.JoystickValues;

public class JoystickSubsystem extends SubsystemBase{
    private CommandJoystick rightJoystick;
    private CommandJoystick leftJoystick;

    private CommandJoystick operatorController;

    public JoystickSubsystem() {
        rightJoystick = new CommandJoystick(Constants.Joystick.RIGHT_JOYSTICK_PORT);
        leftJoystick = new CommandJoystick(Constants.Joystick.LEFT_JOYSTICK_PORT);
        operatorController = new CommandJoystick(Constants.Joystick.OPERATOR_PORT);
    }

    public JoystickValues getRightJoystickValues() {
        return new JoystickValues(rightJoystick.getX(), -rightJoystick.getY());
    }

    public JoystickValues getLeftJoystickValues() {
        return new JoystickValues(leftJoystick.getX(), -leftJoystick.getY());
    }

    public CommandJoystick getRightJoystick() {
        return rightJoystick;
    }

    public CommandJoystick getLeftJoystick() {
        return leftJoystick;
    }

    public boolean getLeftButtonValue(int index) {
        return leftJoystick.button(index).getAsBoolean();
    }
    public boolean getRightButtonValue(int index){
        return rightJoystick.button(index).getAsBoolean();
    }

    public CommandJoystick getOperatorController(){
        return operatorController;
    }
}
