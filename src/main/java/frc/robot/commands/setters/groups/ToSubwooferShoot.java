package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotState;
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
            StateManager.getSetStateCommand(RobotState.SHOOT)

        );
    }
}
