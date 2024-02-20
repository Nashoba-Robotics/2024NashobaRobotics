package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToNeutral;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToNeutral extends SequentialCommandGroup {
    
    public ToNeutral() {
        addCommands(
            new StopAllRollers(),   //Consider adding command to put note into shooter sensor
            new LoaderToNeutral(),
            new NoteToShooter(),
            new ArmToNeutral(),
            StateManager.getSetStateCommand(RobotState.NEUTRAL)
        );
    }

}
