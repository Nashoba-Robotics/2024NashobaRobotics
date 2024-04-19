package frc.robot.commands.setters.units.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmToIntake extends Command {
    
    private ArmSubsystem arm = RobotContainer.arm;

    private Timer t;

    public ArmToIntake() {
        addRequirements(arm);
        t = new Timer();
    }

    @Override
    public void initialize() {
        arm.setArmPivot(Presets.Arm.INTAKE_POS);
        t.restart();
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.isAutonomous() && t.get() > 0.5) return true;
        return arm.getArmPivotAngle().getRadians() < Presets.Arm.ACTUAL_INTAKE_POS.getRadians();
    }

}
