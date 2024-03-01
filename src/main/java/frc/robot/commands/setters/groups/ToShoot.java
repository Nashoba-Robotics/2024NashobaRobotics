package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.arm.ArmMaintainPos;
import frc.robot.commands.setters.units.arm.FinishedShooting;
import frc.robot.commands.setters.units.arm.ShooterToShoot;
import frc.robot.commands.setters.units.intake.IntakeToShoot;
import frc.robot.commands.setters.units.loader.GrabberToShoot;

public class ToShoot extends SequentialCommandGroup {
    
    public ToShoot() {
        addCommands(
            new ArmMaintainPos(),
            Governor.getSetStateCommand(RobotState.SHOOT),
            new ShooterToShoot().withTimeout(2),
            new ParallelCommandGroup(
                new GrabberToShoot()
                // new IntakeToShoot()
            ),
            new FinishedShooting().withTimeout(5)
        );
    }

}
