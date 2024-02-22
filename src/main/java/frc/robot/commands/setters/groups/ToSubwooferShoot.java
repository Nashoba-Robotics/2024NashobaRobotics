package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.commands.setters.units.arm.ArmToIntake;
import frc.robot.commands.setters.units.arm.ArmToNeutral;
import frc.robot.commands.setters.units.arm.ShooterToShoot;
import frc.robot.commands.setters.units.loader.GrabberToShoot;
import frc.robot.commands.setters.units.loader.LoaderToIntake;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;

public class ToSubwooferShoot extends SequentialCommandGroup{
    public ToSubwooferShoot(){
        addCommands(
            new LoaderToNeutral(),
            new ArmToIntake(),
            new LoaderToIntake(),
            new ShooterToShoot(),
            new GrabberToShoot(),
            Governor.getSetStateCommand(RobotState.SHOOT)

        );
    }
}
