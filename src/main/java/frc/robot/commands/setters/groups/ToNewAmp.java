package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setters.units.arm.ArmToAmp;
import frc.robot.commands.setters.units.loader.GrabberToAmp;
import frc.robot.commands.setters.units.loader.LoaderToAmp;
import frc.robot.commands.setters.units.loader.NoteToAmp;
import frc.robot.commands.setters.units.loader.NoteToLoaderOut;

public class ToNewAmp extends SequentialCommandGroup{
    public ToNewAmp(){
        addCommands(
            new ArmToAmp(),
            new NoteToAmp(),
            new LoaderToAmp(),
            new GrabberToAmp()
        );
    }
}
