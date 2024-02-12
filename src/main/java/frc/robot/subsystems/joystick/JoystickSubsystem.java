package frc.robot.subsystems.joystick;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.lib.util.JoystickValues;

public class JoystickSubsystem extends SubsystemBase{

    private JoystickIO io;
    private JoystickIOInputsAutoLogged inputs = new JoystickIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Joysticks", inputs);
    }

    public JoystickSubsystem() {
        io = new JoystickIOSwitchController();
    }

    public JoystickValues getRightJoystickValues() {
        return io.getRightJoystickValues();
    }

    public JoystickValues getLeftJoystickValues() {
        return io.getLeftJoystickValues();
    }

    public JoystickValues getLeftOperatorValues(){
        return new JoystickValues(inputs.operatorJoystickLeftX, inputs.operatorJoystickLeftY);
    }

    public JoystickValues getRightOperatorValues(){
        return new JoystickValues(inputs.operatorJoystickRightX, inputs.operatorJoystickRightY);
    }

    public CommandJoystick getDriverController(){
        return io.getDriverController();
    }
    public CommandJoystick getOperatorController(){
        return io.getOperatorController();
    }

    public boolean driverButtonPressed(int index){
        return io.getDriverController().button(index).getAsBoolean();
    }
    public boolean operatorButtonPressed(int index){
        return io.getOperatorController().button(index).getAsBoolean();
    }
}
