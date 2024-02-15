package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
import frc.robot.commands.setters.units.loader.LoaderToIntake;

public class ToIntake extends SequentialCommandGroup {
    
    public ToIntake() {
        addCommands(
            new LoaderToIntake(),
            // new ArmToIntake(),
            // new ParallelCommandGroup(
            //     new GrabberToIntake(),
            //     new IntakeToIntake()
            // ),
            StateManager.getSetStateCommand(RobotState.INTAKE)
        );
    }

}
