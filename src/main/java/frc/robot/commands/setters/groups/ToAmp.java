package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.arm.ArmToAmp;
import frc.robot.commands.setters.units.loader.GrabberToAmp;
import frc.robot.commands.setters.units.loader.LoaderToAmp;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToLoader;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToAmp extends SequentialCommandGroup {
    
    public ToAmp() {
        addCommands(
            new LoaderToNeutral(),
            new ArmToAmp(),
            new NoteToLoader(),
            new LoaderToAmp(),
            new GrabberToAmp(),
            Governor.getSetStateCommand(RobotState.AMP)
        );
    }

}
