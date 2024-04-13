package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterToShootPrep extends Command{
    ArmSubsystem arm = RobotContainer.arm;

    public  ShooterToShootPrep(){
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // arm.setShooterSpeed(Presets.Arm.SPEAKER_SPEED.div(3).times(2));
        arm.setShooterSpeed(Presets.Arm.SPEAKER_SPEED);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
