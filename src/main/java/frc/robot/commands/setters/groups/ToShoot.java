package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.arm.ArmMaintainPos;
import frc.robot.commands.setters.units.arm.ShooterToShoot;
import frc.robot.commands.setters.units.loader.GrabberToShoot;

public class ToShoot extends SequentialCommandGroup {
    
    public ToShoot() {
        addCommands(
            new ArmMaintainPos(),
            new ShooterToShoot(),
            new GrabberToShoot(),
            Governor.getSetStateCommand(RobotState.SHOOT)
        );
    }

}
