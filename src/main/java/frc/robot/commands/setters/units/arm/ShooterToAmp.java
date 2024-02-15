package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterToAmp extends Command {
    
    private ArmSubsystem arm = RobotContainer.arm;

    public ShooterToAmp() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShooterSpeed(Presets.Arm.AMP_SPEED);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getShooterSpeed().getRadians() - Presets.Arm.AMP_SPEED.getRadians()) < Presets.Arm.SPEED_TOLERANCE.getRadians();
    }

}
