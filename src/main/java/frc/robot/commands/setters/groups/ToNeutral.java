package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToNeutral;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToLoader;
import frc.robot.commands.setters.units.loader.NoteToLoaderOut;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToNeutral extends SequentialCommandGroup {
    
    public ToNeutral() {
        addCommands(
            new StopAllRollers(),
            new NoteToLoaderOut(),
            new LoaderToNeutral(),
            new NoteToShooter(),
            new ArmToNeutral(),
            Governor.getSetStateCommand(RobotState.NEUTRAL)
        );
    }

}
