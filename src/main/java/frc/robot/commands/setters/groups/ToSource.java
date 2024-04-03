package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToSource;
import frc.robot.commands.setters.units.loader.GrabberToSource;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;

public class ToSource extends SequentialCommandGroup {
    
    public ToSource() {
        addCommands(
            new StopAllRollers(),
            new LoaderToNeutral(),
            new ArmToSource(),
            Governor.getSetStateCommand(RobotState.SOURCE),
            new GrabberToSource()
        );
    }

}
