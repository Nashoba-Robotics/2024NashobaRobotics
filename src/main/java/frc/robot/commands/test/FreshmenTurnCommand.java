package frc.robot.commands.test;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class FreshmenTurnCommand extends Command {
   public DriveSubsystem drive=RobotContainer.drive;
   public PIDController controller;
   public FreshmenTurnCommand(){
        controller = new PIDController(0.15, 0.00789, 0.01);//TODO:Put PID values 
        addRequirements(drive);
   }
   @Override
   public void initialize() {
       drive.setGyro(0);
       controller.setSetpoint(30);
   }
   @Override
   public void execute() { 
        ChassisSpeeds kateLikesFeet = new ChassisSpeeds();
        double kateDoesntNotLikeAngledFeet=drive.getYaw().getDegrees();
        double kateLOVESFeet=controller.calculate(kateDoesntNotLikeAngledFeet);
        kateLikesFeet.omegaRadiansPerSecond= kateLOVESFeet;
        drive.set(kateLikesFeet);

        Logger.recordOutput("Freshman Angle thing", kateDoesntNotLikeAngledFeet);
   }
   @Override
   public void end(boolean interrupted) {
       ChassisSpeeds kateREALLYLikesFeet = new ChassisSpeeds();
       kateREALLYLikesFeet.omegaRadiansPerSecond=0;
       drive.set(kateREALLYLikesFeet);
   }
   @Override
   public boolean isFinished() {
       double angle=drive.getYaw().getDegrees();
       if(angle==30){
        return true;
       }
       else{
        return false;
       }
       
   }
}
