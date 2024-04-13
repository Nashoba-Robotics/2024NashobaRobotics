package frc.robot.commands.setters.groups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToClimb;
import frc.robot.commands.setters.units.climber.ClimberToClimb;
import frc.robot.commands.setters.units.climber.ClimberToManual;
import frc.robot.commands.setters.units.loader.LoaderToClimb;

public class ToClimb extends SequentialCommandGroup {
    
    public ToClimb() {
        addCommands(
            new StopAllRollers(),
            // new ArmToClimb(),
            new ClimberToClimb().withTimeout(5),
            Governor.getSetStateCommand(RobotState.CLIMB),
            new ClimberToManual()
        );
    }

}
