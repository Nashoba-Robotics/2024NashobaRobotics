package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterToShuttle extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    Rotation2d speed = Rotation2d.fromRadians(Presets.Arm.SHUTTLE_SPEED.getRadians() + 10);

    public ShooterToShuttle(){
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setShooterSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        // return arm.getShooterSpeed().getRadians() >= speed.getRadians();
        return true;
    }
}
