package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToAmp;
import frc.robot.commands.setters.units.arm.ShooterToAmp;
import frc.robot.commands.setters.units.loader.GrabberToAmp;
import frc.robot.commands.setters.units.loader.LoaderToAmp;

public class ToAmpAdj extends SequentialCommandGroup {
    
    public ToAmpAdj() {
        addCommands(
            new StopAllRollers(),
            new LoaderToAmp(),
            new ArmToAmp(),
            Governor.getSetStateCommand(RobotState.AMP_ADJ),
            new ShooterToAmp()
        );
    }

}
