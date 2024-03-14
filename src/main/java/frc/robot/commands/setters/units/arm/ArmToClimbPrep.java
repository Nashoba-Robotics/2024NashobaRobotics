package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmToClimbPrep extends Command {
    
    private ArmSubsystem arm = RobotContainer.arm;

    public ArmToClimbPrep() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmPivot(Presets.Arm.CLIMB_PREP_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Presets.Arm.CLIMB_PREP_POS.getRadians() - arm.getArmPivotAngle().getRadians()) < Presets.Arm.POS_TOLERANCE.getRadians();
    }

}
