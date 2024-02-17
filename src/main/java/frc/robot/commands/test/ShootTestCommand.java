package frc.robot.commands.test;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class ShootTestCommand extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    LoaderSubsystem loader = RobotContainer.loader;
    DriveSubsystem drive = RobotContainer.drive;

    private final double MAX_SHOOT_SPEED = 500;

    public ShootTestCommand(){
        addRequirements(arm, loader);
    }

    @Override
    public void execute() {
        double y = Constants.Field.getSpeakerPos().getZ()-Constants.Robot.SHOOTER_HEIGHT;
        // Translation2d shootPos = new Translation2d(drive.getPose().getX(), y)
        double dist = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
        dist -= 0.22;
        double angle = -Math.atan2(y, dist);
        
        Logger.recordOutput("Shoot Dist", dist);
        Logger.recordOutput("Estimated Angle", angle*360/Constants.TAU);

        double targetAngle = SmartDashboard.getNumber("Arm Angle", 0);
        arm.setArmPivot(Rotation2d.fromDegrees(targetAngle));

        double loaderSpeed = SmartDashboard.getNumber("Loader Speed", 0);
        loader.setRollerSpeed(loaderSpeed);
    }
}
