package frc.robot.commands.setters.groups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToNeutral;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.NoteToShooter;

public class ToNeutral extends SequentialCommandGroup {
    private LoaderToNeutral lNeut;
    private NoteToShooter nShoot;
    public ToNeutral() {
        lNeut = new LoaderToNeutral();
        nShoot = new NoteToShooter();
        addCommands(
            new InstantCommand(() -> RobotContainer.overrideAngle = false),
            new StopAllRollers(),
            new InstantCommand(()->Governor.cleanUp=false),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    lNeut,
                    nShoot
                ),
                new SequentialCommandGroup(    
                    new InstantCommand(()->
                    {
                        if(RobotContainer.arm.getArmPivotAngle().getDegrees() > -35)
                            RobotContainer.arm.setArmPivot(Rotation2d.fromDegrees(-35));
                    }
                    ),
                    new WaitUntilCommand(new BooleanSupplier() {
                        public boolean getAsBoolean(){
                            return nShoot.isFinished(); //TODO: FBP
                        }
                    }),
                    new ArmToNeutral()
                )
            ),
            Governor.getSetStateCommand(RobotState.NEUTRAL)
        );
    }

    // public ToNeutral(boolean override) {
    //     addCommands(
    //         new StopAllRollers(),
    //         new NoteToLoaderOut(),
    //         new LoaderToNeutral(),
    //         new NoteToShooter(override),
    //         new ArmToNeutral(),
    //         Governor.getSetStateCommand(RobotState.NEUTRAL)
    //     );
    // }

}
