package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToIntake;
import frc.robot.commands.setters.units.loader.LoaderToIntake;
import frc.robot.commands.setters.units.loader.NoteToLoader;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToIntakeAdj extends SequentialCommandGroup {
    
    public ToIntakeAdj() {
        addCommands(
            new StopAllRollers(),
            new LoaderToIntake(),
            new ArmToIntake(),
            Governor.getSetStateCommand(RobotState.INTAKE_ADJ)
        );
    }

}