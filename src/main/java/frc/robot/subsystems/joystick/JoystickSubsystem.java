package frc.robot.subsystems.joystick;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
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
        io = new JoystickIOThrustMaster();
    }

    public JoystickValues getRightJoystickValues() {
        return new JoystickValues(inputs.rightJoystickX, inputs.rightJoystickY);
    }

    public JoystickValues getLeftJoystickValues() {
        return new JoystickValues(inputs.leftJoystickX, inputs.leftJoystickY);
    }

    public CommandJoystick getRightJoystick() {
        return io.getRightJoystick();
    }

    public CommandJoystick getLeftJoystick() {
        return io.getLeftJoystick();
    }

    public boolean getLeftButtonValue(int index) {
        return getLeftJoystick().button(index).getAsBoolean();
    }
    public boolean getRightButtonValue(int index){
        return getRightJoystick().button(index).getAsBoolean();
    }

    public CommandJoystick getOperatorController(){
        return io.getOperatorController();
    }
}
