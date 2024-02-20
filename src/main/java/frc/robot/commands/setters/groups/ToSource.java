package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.arm.ArmToSource;
import frc.robot.commands.setters.units.loader.GrabberToSource;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToLoaderOut;

public class ToSource extends SequentialCommandGroup {
    
    public ToSource() {
        addCommands(
            new LoaderToNeutral(),  //Figure out a way to check if we are already at source position
            new ArmToSource(),
            // new LoaderToSource(),
            new GrabberToSource(),
            new NoteToLoaderOut(),
            Governor.getSetStateCommand(RobotState.SOURCE)
        );
    }

}
