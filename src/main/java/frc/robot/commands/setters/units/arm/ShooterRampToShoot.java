package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterRampToShoot extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    double targetSpeed, startSpeed, calculatedSpeed;
    double startTime;
    final double rampTime = 3;

    public ShooterRampToShoot(){
        this.targetSpeed = Presets.Arm.SPEAKER_SPEED.getRadians();
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis()/1000;
        startSpeed = arm.getShooterSpeed().getRadians();
    }

    @Override
    public void execute() {
        double time = (System.currentTimeMillis() - startTime);

        calculatedSpeed = (startSpeed + (Presets.Arm.SPEAKER_SPEED.getRadians()-startSpeed)/rampTime * time);

        arm.setShooterSpeed(Rotation2d.fromRadians(calculatedSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShooterSpeed(Rotation2d.fromRadians(targetSpeed));
    }

    @Override
    public boolean isFinished() {
        return startSpeed >= targetSpeed || calculatedSpeed >= targetSpeed;
    }
}
