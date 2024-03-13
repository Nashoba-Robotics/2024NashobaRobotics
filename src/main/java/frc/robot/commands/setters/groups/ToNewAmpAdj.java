package frc.robot.commands.setters.groups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToAmp;
import frc.robot.commands.setters.units.loader.LoaderToAmp;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToAmpIn;
import frc.robot.commands.setters.units.loader.NoteToAmpOut;
import frc.robot.commands.setters.units.loader.NoteToLoaderOut;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToNewAmpAdj extends SequentialCommandGroup{
    public ToNewAmpAdj(){
        addCommands(
            new StopAllRollers(),
            new ParallelCommandGroup(
                new ArmToAmp(),
                new SequentialCommandGroup(
                    new WaitUntilCommand(new BooleanSupplier() {
                        public boolean getAsBoolean(){
                            return RobotContainer.arm.getArmPivotAngle().getDegrees() >= -30;
                        }
                    }),
                    new NoteToAmpOut(),
                    new LoaderToAmp()
                    // new NoteToAmpIn()
                    
                )
            ),
            Governor.getSetStateCommand(RobotState.AMP_ADJ)
        );
    }
}
