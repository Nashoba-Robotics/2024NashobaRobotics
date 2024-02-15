package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToSource;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.LoaderToSource;

public class ToSourceAdj extends SequentialCommandGroup {
 
    public ToSourceAdj() {
        addCommands(
            new StopAllRollers(),
            new LoaderToNeutral(),
            new ArmToSource(),
            new LoaderToSource(),
            StateManager.getSetStateCommand(RobotState.SOURCE_ADJ)
        );
    }

}
