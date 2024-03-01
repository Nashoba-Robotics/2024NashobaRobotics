package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
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
        // arm.setShooterSpeed(Presets.Arm.SPEAKER_SPEED.plus(Rotation2d.fromRadians(10)));
        Rotation2d speed = Rotation2d.fromRadians(Presets.Arm.SPEAKER_SPEED.getRadians() + 20);
        // arm.setShooterSpeed(Presets.Arm.SPEAKER_SPEED.plus(Rotation2d.fromRadians(20)));
        arm.setShooterSpeed(speed);


        // arm.setShooterPercent(Presets.Arm.SPEAKER_PERCENT);
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(arm.getShooterSpeed().getRadians()-Presets.Arm.SPEAKER_SPEED.getRadians()) <= Presets.Arm.SPEED_TOLERANCE.getRadians();
        return Math.abs(arm.getShooterSpeed().getRadians()) >= Presets.Arm.SPEAKER_SPEED.getRadians() || DriverStation.isAutonomous();
                // || timer.get() > 2; 

        // return true;
    }
}
