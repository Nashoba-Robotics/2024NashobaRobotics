package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterToShoot extends Command{
    ArmSubsystem arm = RobotContainer.arm;

    public ShooterToShoot(){
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // arm.setShooterSpeed(Presets.Arm.SPEAKER_SPEED);
        arm.setShooterPercent(Presets.Arm.SPEAKER_PERCENT);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getShooterSpeed().getRadians()) >= 300;
        // return true;
    }
}