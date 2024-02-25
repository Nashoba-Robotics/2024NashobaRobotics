package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmMaintainPos extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    Rotation2d currentPos;
    public ArmMaintainPos(){
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        currentPos = arm.getArmPivotAngle();
    }

    @Override
    public void execute() {
        arm.setArmPivot(currentPos);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getArmPivotAngle().getRadians() - currentPos.getRadians()) <= Presets.Arm.POS_TOLERANCE.getRadians();

    }
}
