package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;

public class ToShootPrep extends SequentialCommandGroup {
    
    public ToShootPrep() {
        addCommands(
            StateManager.getSetStateCommand(RobotState.SHOOT_PREP)
        );
    }

}
