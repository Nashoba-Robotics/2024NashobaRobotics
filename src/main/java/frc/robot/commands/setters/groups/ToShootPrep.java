package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToShoot;
import frc.robot.commands.setters.units.arm.ArmToShootPrep;
import frc.robot.commands.setters.units.arm.ShooterToShootPrep;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToShootPrep extends SequentialCommandGroup {
    
    public ToShootPrep() {
        addCommands(
            new StopAllRollers(),
            new LoaderToNeutral(),
            new NoteToShooter(),
            new ShooterToShootPrep(),
            Governor.getSetStateCommand(RobotState.SHOOT_PREP),
            new ArmToShootPrep()
        );
    }

    public ToShootPrep(boolean move){
        addCommands(
            Governor.getSetStateCommand(RobotState.TRANSITION),
            new StopAllRollers(),
            new LoaderToNeutral(),
            new NoteToShooter(),
            new ShooterToShootPrep(move),
            Governor.getSetStateCommand(RobotState.SHOOT_PREP),
            new ArmToShootPrep()
        );
    }

}
