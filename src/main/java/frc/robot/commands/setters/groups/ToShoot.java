package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
import frc.robot.commands.setters.units.arm.ArmMaintainPos;
import frc.robot.commands.setters.units.arm.ShooterToShoot;
import frc.robot.commands.setters.units.loader.GrabberToShoot;

public class ToShoot extends SequentialCommandGroup {
    
    public ToShoot() {
        addCommands(
            new ArmMaintainPos(),
            new ShooterToShoot(),
            new GrabberToShoot(),
            StateManager.getSetStateCommand(RobotState.SHOOT)
        );
    }

}
