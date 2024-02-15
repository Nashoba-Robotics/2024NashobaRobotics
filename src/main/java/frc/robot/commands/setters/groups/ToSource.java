package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
import frc.robot.commands.setters.units.arm.ArmToSource;
import frc.robot.commands.setters.units.loader.GrabberToSource;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.LoaderToSource;

public class ToSource extends SequentialCommandGroup {
    
    public ToSource() {
        addCommands(
            new LoaderToNeutral(),  //Figure out a way to check if we are already at source position
            new ArmToSource(),
            new LoaderToSource(),
            new GrabberToSource(),
            StateManager.getSetStateCommand(RobotState.SOURCE)
        );
    }

}
