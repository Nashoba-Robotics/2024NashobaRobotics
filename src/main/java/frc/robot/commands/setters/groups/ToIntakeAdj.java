package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;

public class ToIntakeAdj extends SequentialCommandGroup {
    
    public ToIntakeAdj() {
        addCommands(
            StateManager.getSetStateCommand(RobotState.INTAKE_ADJ)
        );
    }

}