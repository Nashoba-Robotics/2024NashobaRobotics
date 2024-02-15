package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;

public class ToIntake extends SequentialCommandGroup {
    
    public ToIntake() {
        addCommands(
            StateManager.getSetStateCommand(RobotState.INTAKE)
        );
    }

}
