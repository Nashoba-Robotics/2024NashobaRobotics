package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
import frc.robot.commands.setters.units.arm.ArmToIntake;
import frc.robot.commands.setters.units.intake.IntakeToIntake;
import frc.robot.commands.setters.units.loader.GrabberToIntake;
import frc.robot.commands.setters.units.loader.LoaderToIntake;
import frc.robot.commands.setters.units.loader.NoteToLoader;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToIntake extends SequentialCommandGroup {
    
    public ToIntake() {
        addCommands(
            new LoaderToIntake(),
            new ArmToIntake(),
            new ParallelCommandGroup(
                new GrabberToIntake(),
                new IntakeToIntake(),
                StateManager.getSetStateCommand(RobotState.INTAKE)
            )
        );
    }

}
