package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToIntake;
import frc.robot.commands.setters.units.loader.LoaderToIntake;

public class ToIntakeAdj extends SequentialCommandGroup {
    
    public ToIntakeAdj() {
        addCommands(
            new StopAllRollers(),
            new LoaderToIntake(),
            new ArmToIntake(),
            StateManager.getSetStateCommand(RobotState.INTAKE_ADJ)
        );
    }

}