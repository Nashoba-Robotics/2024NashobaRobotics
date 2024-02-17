package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setters.units.arm.ArmToShoot;
import frc.robot.commands.setters.units.arm.ArmToW0;
import frc.robot.commands.setters.units.arm.ShooterToShoot;
import frc.robot.commands.setters.units.loader.GrabberToShoot;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class W0ShootCommand extends SequentialCommandGroup{
    public W0ShootCommand(){
        addCommands(
            new LoaderToNeutral(),
            new NoteToShooter(),
            new ArmToW0(),
            new ParallelCommandGroup(
                // new AimToSpeakerCommand(),
                
                new ShooterToShoot()
            ),
            new GrabberToShoot()
        );
    }
}
