package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmToIntake extends Command {
    
    private ArmSubsystem arm = RobotContainer.arm;

    public ArmToIntake() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmPivot(Presets.Arm.INTAKE_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getArmPivotAngle().getRadians() - Presets.Arm.INTAKE_POS.getRadians()) < Presets.Arm.POS_TOLERANCE.getRadians();
    }

}
