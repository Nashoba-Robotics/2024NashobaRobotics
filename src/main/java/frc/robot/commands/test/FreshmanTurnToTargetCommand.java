package frc.robot.commands.test;

import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.drive.DriveSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class FreshmanTurnToTargetCommand extends Command{
    public DriveSubsystem drive = RobotContainer.drive;
    public PIDController controller; 
    public double lastKnownTarget = drive.getGyroAngle().getRadians();
    public FreshmanTurnToTargetCommand(){
        controller = new PIDController(3.7, 0, 0.003);
        addRequirements(drive);
    }

    public void initialize(){
      double spinnyaround = AprilTagManager.getTargetYaw();
        controller.setSetpoint(0);
    
    }

    public void execute(){
        ChassisSpeeds somthingAboutSpeed = new ChassisSpeeds();
        double targetAngle = AprilTagManager.getTargetYaw();
        double PIDinputButItsACoolName = controller.calculate (targetAngle);
        // run the PID on the last known target.
        double PIDLastKnownTarget = controller.calculate (lastKnownTarget);
        if(AprilTagManager.hasTarget()) somthingAboutSpeed.omegaRadiansPerSecond = PIDinputButItsACoolName;
        // else somthingAboutSpeed.omegaRadiansPerSecond = Math.signum(PIDLastKnownTarget) * 0.5; 
        else somthingAboutSpeed.omegaRadiansPerSecond = PIDinputButItsACoolName - drive.getGyroAngle().getRadians();

        drive.set(somthingAboutSpeed);
         Logger.recordOutput ("Freshmen Angle Thing", PIDinputButItsACoolName);
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds noMoreSpeed = new ChassisSpeeds();
        noMoreSpeed.omegaRadiansPerSecond = 0;
        drive.set (noMoreSpeed);
    }
    
    @Override
    public boolean isFinished() {
        // double angle = AprilTagManager.getTargetYaw();
        // if (angle==0) return true;
        // else return false;

        return false;
    }
    
}