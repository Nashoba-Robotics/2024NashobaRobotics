package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToShuttle;
import frc.robot.commands.setters.units.arm.ShooterToShuttle;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToShuttlePrep extends SequentialCommandGroup{
    public ToShuttlePrep(boolean high){
        addCommands(
            new StopAllRollers(),
            new LoaderToNeutral(),
            new NoteToShooter(),
            new InstantCommand(()->RobotContainer.arm.setShooterSpeed(Presets.Arm.SHUTTLE_SPEED), RobotContainer.arm),
            new ArmToShuttle(high),
            Governor.getSetStateCommand(RobotState.SHUTTLE_ADJ),
            new ShooterToShuttle()
        );
    }
}
