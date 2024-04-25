package frc.robot.subsystems.joystick;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.lib.util.JoystickValues;

public class JoystickIOSwitchController implements JoystickIO{
    CommandJoystick driveController;
    CommandJoystick operatorController;
    public JoystickIOSwitchController(){
        driveController = new CommandJoystick(Constants.Joystick.DRIVER_PORT);
        operatorController = new CommandJoystick(Constants.Joystick.OPERATOR_PORT);
    }

    public void updateInputs(JoystickIOInputs inputs){
        inputs.driveLeftJoystickX = driveController.getX();
        inputs.driveLeftJoystickY = -driveController.getY();
        inputs.driveRightJoystickX = driveController.getZ();
        inputs.driveRightJoystickY = -driveController.getThrottle();

        inputs.operatorJoystickLeftX = operatorController.getX();
        inputs.operatorJoystickLeftY = -operatorController.getY();
        inputs.operatorJoystickRightX = operatorController.getZ();
        inputs.operatorJoystickRightY = -operatorController.getThrottle();
    }

    @Override
    public JoystickValues getLeftJoystickValues(){
        int multiplier = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 1 : 1;
        return new JoystickValues(driveController.getX() * multiplier, driveController.getY() * multiplier);
    }
    @Override
    public JoystickValues getRightJoystickValues(){
        return new JoystickValues(driveController.getZ(), -driveController.getTwist());
    }
    @Override
    public CommandJoystick getDriverController(){
        return driveController;
    }
    @Override
    public CommandJoystick getOperatorController(){
        return operatorController;
    }
}
