package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterRampToIdle extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    double targetSpeed, startSpeed, calculatedSpeed;
    double startTime;
    final double rampTime = 3;

    public ShooterRampToIdle(double idleSpeedPercent){
        this.targetSpeed = idleSpeedPercent *= 500;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis()/1000;
        startSpeed = arm.getShooterSpeed().getRadians();
    }

    @Override
    public void execute() {
        double time = System.currentTimeMillis()/1000 - startTime;

        double calculatedSpeed = startSpeed - (startSpeed-targetSpeed)/rampTime * time;

        arm.setShooterSpeed(Rotation2d.fromRadians(calculatedSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShooterPercent(targetSpeed/500);
    }

    @Override
    public boolean isFinished() {
        return startSpeed <= targetSpeed || calculatedSpeed <= targetSpeed;
    }
}
