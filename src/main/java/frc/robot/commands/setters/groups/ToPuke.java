package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToNeutral;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToPuke extends SequentialCommandGroup{
    public ToPuke(){
        addCommands(
            new StopAllRollers(),
            new LoaderToNeutral(),
            new NoteToShooter(),
            new ArmToNeutral(),
            Governor.getSetStateCommand(RobotState.MISC),
            new ParallelCommandGroup(
                new InstantCommand(()->RobotContainer.intake.setSpeed(-Presets.Intake.INTAKE_SPEED), RobotContainer.intake),
                new InstantCommand(() -> RobotContainer.loader.setRollerSpeed(-Presets.Loader.INTAKE_SPEED), RobotContainer.loader)
            )
        );
    }
}
