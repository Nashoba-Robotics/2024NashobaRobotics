package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.arm.ArmMaintainPos;
import frc.robot.commands.setters.units.arm.ArmToShoot;
import frc.robot.commands.setters.units.arm.ShooterToShoot;
import frc.robot.commands.setters.units.loader.GrabberToShoot;

public class ToShoot extends SequentialCommandGroup {
    
    public ToShoot() {
        addCommands(
            new InstantCommand(() -> RobotContainer.loader.setRollerSpeed(0), RobotContainer.loader),
            new ArmToShoot().withTimeout(2),
            // new ArmMaintainPos(),
            // new ShooterToShoot().withTimeout(0.7),
            new ShooterToShoot(),
            Governor.getSetStateCommand(RobotState.SHOOT),
            new ParallelCommandGroup(
                new GrabberToShoot()
            )
        );
    }

}
