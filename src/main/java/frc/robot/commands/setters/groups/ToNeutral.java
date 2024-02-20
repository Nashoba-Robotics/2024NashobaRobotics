package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToNeutral;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;

public class ToNeutral extends SequentialCommandGroup {
    
    public ToNeutral() {
        addCommands(
            new StopAllRollers(),   //Consider adding command to put note into shooter sensor
            new LoaderToNeutral(),
            new ArmToNeutral(),
            Governor.getSetStateCommand(RobotState.NEUTRAL)
        );
    }

}
