package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmToW0 extends Command{
    ArmSubsystem arm = RobotContainer.arm;

    public ArmToW0(){
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setArmPivot(Presets.Arm.PODIUM_SHOOTER_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getArmPivotAngle().getRadians() - Presets.Arm.PODIUM_SHOOTER_POS.getRadians()) <= Presets.Arm.POS_TOLERANCE.getRadians(); 
    }
}
