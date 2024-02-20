package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
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
            StateManager.getSetStateCommand(RobotState.AMP)
        );
    }

}
