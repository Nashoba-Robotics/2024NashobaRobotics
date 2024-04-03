package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToIntake;
import frc.robot.commands.setters.units.intake.IntakeToIntake;
import frc.robot.commands.setters.units.loader.GrabberToIntake;
import frc.robot.commands.setters.units.loader.LoaderToIntake;

public class ToIntake extends SequentialCommandGroup {
    
    public ToIntake() {
        addCommands(
            new StopAllRollers(),
            new ParallelCommandGroup(
                new LoaderToIntake(),
                new ArmToIntake()
            ),
            new GrabberToIntake(),
            new IntakeToIntake(),
            Governor.getSetStateCommand(RobotState.INTAKE)
        );
    }

}
