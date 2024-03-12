package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.arm.ArmToShuttle;
import frc.robot.commands.setters.units.arm.ShooterToShuttle;
import frc.robot.commands.setters.units.loader.GrabberToShuttle;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToShuttle extends SequentialCommandGroup{
    public ToShuttle(){
        addCommands(
            new LoaderToNeutral(),
            new NoteToShooter(),
            new ArmToShuttle(),
            new ShooterToShuttle(),
            Governor.getSetStateCommand(RobotState.SHUTTLE),
            new GrabberToShuttle()
        );
    }
}
