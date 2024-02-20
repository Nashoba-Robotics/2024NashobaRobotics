package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.arm.ArmToAmp;
import frc.robot.commands.setters.units.loader.LoaderToAmp;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToLoader;
import frc.robot.commands.setters.units.loader.NoteToLoaderOut;

public class ToAmpAdj extends SequentialCommandGroup {
    
    public ToAmpAdj() {
        addCommands(
            new LoaderToNeutral(),
            new NoteToLoader(),
            new ArmToAmp(),
            new LoaderToAmp(),
            Governor.getSetStateCommand(RobotState.AMP_ADJ)
        );
    }

}
