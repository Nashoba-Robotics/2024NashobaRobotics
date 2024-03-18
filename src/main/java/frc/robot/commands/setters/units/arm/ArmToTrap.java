package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmToTrap extends Command {
    
    private ArmSubsystem arm = RobotContainer.arm;

    public ArmToTrap() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmPivot(Presets.Arm.TRAP_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getArmPivotAngle().getRadians() - Presets.Arm.TRAP_POS.getRadians()) < Presets.Arm.POS_TOLERANCE.getRadians();
    }

}
