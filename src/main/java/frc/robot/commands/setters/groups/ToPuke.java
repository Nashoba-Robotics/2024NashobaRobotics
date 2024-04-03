package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;

public class ToPuke extends SequentialCommandGroup{
    public ToPuke(){
        addCommands(
            Governor.getSetStateCommand(RobotState.MISC),
            new ParallelCommandGroup(
                new InstantCommand(()->RobotContainer.intake.setSpeed(-Presets.Intake.INTAKE_SPEED), RobotContainer.intake),
                new InstantCommand(() -> RobotContainer.loader.setRollerSpeed(-Presets.Loader.INTAKE_SPEED), RobotContainer.loader)
            )
        );
    }
}
