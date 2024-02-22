package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
        arm.setShooterSpeed(Presets.Arm.SPEAKER_SPEED_PREP);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
