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
    public double lastKnownYaw;
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
        double targetAngleSpeedOrSomething = AprilTagManager.getTargetYaw();
        double PIDinputButItsACoolName = controller.calculate (targetAngleSpeedOrSomething);
        lastKnownYaw = AprilTagManager.getTargetYaw();
        double PIDinputLastKnownYaw = controller.calculate (lastKnownYaw);
        if(AprilTagManager.hasTarget()) somthingAboutSpeed.omegaRadiansPerSecond = PIDinputButItsACoolName;
        else somthingAboutSpeed.omegaRadiansPerSecond = Math.signum(PIDinputLastKnownYaw) * 0.5;   //Math.signum(double)

        drive.set(somthingAboutSpeed);
         Logger.recordOutput ("Freshmen Angle Thing", targetAngleSpeedOrSomething);
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