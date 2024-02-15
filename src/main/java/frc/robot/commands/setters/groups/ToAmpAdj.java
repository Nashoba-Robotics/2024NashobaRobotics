package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;

public class ToAmpAdj extends SequentialCommandGroup {
    
    public ToAmpAdj() {
        addCommands(
            StateManager.getSetStateCommand(RobotState.AMP_ADJ)
        );
    }

}
