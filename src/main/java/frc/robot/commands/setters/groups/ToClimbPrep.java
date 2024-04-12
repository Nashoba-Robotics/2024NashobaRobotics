package frc.robot.commands.setters.groups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToClimbPrep;
import frc.robot.commands.setters.units.climber.ClimberToManual;
import frc.robot.commands.setters.units.climber.ClimberToClimbPrep;
import frc.robot.commands.setters.units.loader.LoaderToClimbPrep;
import frc.robot.commands.setters.units.loader.NoteToAmpOut;

public class ToClimbPrep extends SequentialCommandGroup {
    
    public ToClimbPrep() {
        addCommands(

            new ClimberToClimbPrep(),
            new ClimberToManual()
        );
    }

}
