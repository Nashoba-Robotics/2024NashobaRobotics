package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToShoot;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;

public class ToShootPrep extends SequentialCommandGroup {
    
    public ToShootPrep() {
        addCommands(
            new StopAllRollers(),
            new ParallelCommandGroup(
                new ArmToShoot(),
                StateManager.getSetStateCommand(RobotState.SHOOT_PREP)
            )
        );
    }

}
