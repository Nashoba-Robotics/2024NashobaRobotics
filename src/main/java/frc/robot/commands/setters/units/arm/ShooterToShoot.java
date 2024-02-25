package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterToShoot extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    Timer timer;

    public 
    ShooterToShoot(){
        addRequirements(arm);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        arm.setShooterSpeed(Presets.Arm.SPEAKER_SPEED);
        // arm.setShooterPercent(Presets.Arm.SPEAKER_PERCENT);
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(arm.getShooterSpeed().getRadians()-Presets.Arm.SPEAKER_SPEED.getRadians()) <= Presets.Arm.SPEED_TOLERANCE.getRadians();
        return Math.abs(arm.getShooterSpeed().getRadians()) >= Presets.Arm.SPEAKER_SPEED.getRadians() || DriverStation.isAutonomous(); 

        // return true;
    }
}
