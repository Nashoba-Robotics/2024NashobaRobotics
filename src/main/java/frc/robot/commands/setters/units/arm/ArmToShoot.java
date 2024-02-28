package frc.robot.commands.setters.units.arm;

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

public class ArmToShoot extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    DriveSubsystem drive = RobotContainer.drive;

    double angle;

    public ArmToShoot(){
        angle = 0;

        addRequirements(arm);
    }
    @Override
    public void execute() {
        
        double dist = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
        double angle = 0;
        if(RobotContainer.drive.getPose().getY() > 4.04) {
            angle = -0.435322 * Math.atan(-0.797911*dist + 1.42314) - 0.768085;
        } else {
            angle = -0.516972 * Math.atan(-1.29721*dist + 2.25625) - 0.962371;
        }
       // TODO: Check if the angle is within our domain. 
        arm.setArmPivot(Rotation2d.fromRadians(angle).plus(Presets.Arm.SPEAKER_OFFSET));    //Adds on Operator Input
    }
    @Override
    public boolean isFinished() {
        if(DriverStation.isAutonomous()) return Math.abs(arm.getArmPivotAngle().getRadians()-angle) <= Presets.Arm.POS_TOLERANCE.getRadians();
        return Governor.getRobotState() != RobotState.SHOOT_PREP;
    }
}
