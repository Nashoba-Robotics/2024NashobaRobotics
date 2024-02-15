package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;

public class ToSourceAdj extends SequentialCommandGroup {
 
    public ToSourceAdj() {
        addCommands(
            StateManager.getSetStateCommand(RobotState.SOURCE_ADJ)
        );
    }

}
