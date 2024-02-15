package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
import frc.robot.commands.setters.units.arm.ArmToAmp;
import frc.robot.commands.setters.units.loader.LoaderToAmp;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;

public class ToAmpAdj extends SequentialCommandGroup {
    
    public ToAmpAdj() {
        addCommands(
            new LoaderToNeutral(),
            new ArmToAmp(),
            new LoaderToAmp(),
            StateManager.getSetStateCommand(RobotState.AMP_ADJ)
        );
    }

}
