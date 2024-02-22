package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmToSource extends Command{
    ArmSubsystem arm = RobotContainer.arm;

    public ArmToSource(){
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if(!RobotContainer.loader.getLoaderSensor() && !RobotContainer.loader.getShooterSensor()) arm.setArmPivot(Presets.Arm.SOURCE_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getArmPivotAngle().getRadians() - Presets.Arm.SOURCE_POS.getRadians()) <= Presets.Arm.POS_TOLERANCE.getRadians()
        || RobotContainer.loader.getLoaderSensor() || RobotContainer.loader.getShooterSensor();
    }
}
