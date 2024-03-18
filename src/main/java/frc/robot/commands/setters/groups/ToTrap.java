package frc.robot.commands.setters.groups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.arm.ArmToTrap;
import frc.robot.commands.setters.units.climber.ClimberToTrap;
import frc.robot.commands.setters.units.loader.GrabberToTrap;
import frc.robot.commands.setters.units.loader.LoaderToTrap;

public class ToTrap extends SequentialCommandGroup {
    
    public ToTrap() {
        addCommands(
            new ParallelRaceGroup(
                new WaitUntilCommand(new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                        return Governor.getLastRobotState() != RobotState.CLIMB;
                    }
                }),
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new ArmToTrap(),
                            new LoaderToTrap(),
                            new ClimberToTrap()
                        ),
                        new GrabberToTrap()
                    ),
                    Governor.getSetStateCommand(RobotState.TRAP)
                )
            ),
            new InstantCommand(() -> {
                if(Governor.getLastRobotState() != RobotState.TRAP) Governor.setRobotState(RobotState.NEUTRAL, true);
            })
        );
    }

}
