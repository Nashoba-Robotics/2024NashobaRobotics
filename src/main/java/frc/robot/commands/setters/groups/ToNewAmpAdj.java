package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setters.units.arm.ArmToAmp;
import frc.robot.commands.setters.units.loader.LoaderToAmp;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToAmp;
import frc.robot.commands.setters.units.loader.NoteToLoaderOut;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToNewAmpAdj extends SequentialCommandGroup{
    public ToNewAmpAdj(){
        addCommands(
            new LoaderToNeutral(),
            new NoteToShooter(),
            new ArmToAmp(),
            new NoteToAmp(),
            new LoaderToAmp()
        );
    }
}
