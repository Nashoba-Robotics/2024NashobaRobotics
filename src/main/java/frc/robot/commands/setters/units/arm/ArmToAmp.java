package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmToAmp extends Command {
    
    private ArmSubsystem arm = RobotContainer.arm;

    public ArmToAmp() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmPivot(Presets.Arm.AMP_POS);
    }

    @Override
    public boolean isFinished() {
<<<<<<< HEAD
        return Math.abs(arm.getArmPivotAngle().getRadians() - Presets.Arm.AMP_POS.getRadians()) < Presets.Arm.AMP_POS_TOLERANCE.getRadians(); 
=======
        return Math.abs(arm.getArmPivotAngle().getRadians() - Presets.Arm.AMP_POS.getRadians()) < 0.05; 
>>>>>>> bfd02cea78a316241c4434205d8cb26349dda1b7
    }

}
