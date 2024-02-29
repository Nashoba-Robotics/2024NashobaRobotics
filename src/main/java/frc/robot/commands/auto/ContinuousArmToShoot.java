package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Governor;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ContinuousArmToShoot extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    DriveSubsystem drive = RobotContainer.drive;

    double angle;

    public ContinuousArmToShoot(){
        angle = 0;

        addRequirements(arm);
    }
    @Override
    public void execute() {
        
        double dist = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
        angle = 0;
        // if(RobotContainer.drive.getPose().getY() > 4.04) {
            angle = 0.925033 * Math.atan(0.673009*dist) - 1.57125 + 0.001;   //TODO: Add operator offset
        // angle = 0.435322 * Math.atan(0.797911*dist - 1.42314) - 0.768085;   //TODO: Add operator offset

        // } else {
        //     angle = -0.516972 * Math.atan(-1.29721*dist + 2.25625) - 0.962371;
        // }
       // TODO: Check if the angle is within our domain. 
        arm.setArmPivot(Rotation2d.fromRadians(angle));    //Adds on Operator Input
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
