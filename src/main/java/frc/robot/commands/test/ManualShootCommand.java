package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class ManualShootCommand extends Command {
    
    private LoaderSubsystem loader;
    private ArmSubsystem arm;

    public ManualShootCommand(LoaderSubsystem loader, ArmSubsystem arm) {
        this.loader = loader;
        this.arm = arm;
    }

    @Override
    public void initialize() {
        Governor.setRobotState(RobotState.MISC, true);

        SmartDashboard.putNumber("ShooterSpeed", 0);
        SmartDashboard.putNumber("LoaderSpeed", 0);
        SmartDashboard.putNumber("ArmAngle", arm.getArmPivotAngle().getDegrees());
        SmartDashboard.putNumber("LoaderAngle", loader.getPivotAngle().getDegrees());
    }

    @Override
    public void execute() {
        loader.setRollerSpeed(SmartDashboard.getNumber("LoaderSpeed", 0));
        loader.setPivot(Rotation2d.fromDegrees(SmartDashboard.getNumber("LoaderAngle", 0)));
        arm.setShooterSpeed(Rotation2d.fromRadians(SmartDashboard.getNumber("ShooterSpeed", 0)));
        arm.setArmPivot(Rotation2d.fromDegrees(SmartDashboard.getNumber("ArmAngle", 0)));
    }

}
