package frc.robot.commands.setters.units.arm;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.lib.util.DistanceToArmAngleModel;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

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
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            if(RobotContainer.drive.getPose().getX() < Constants.Misc.CLOSE_FAR_CUTOFF) {
                angle = DistanceToArmAngleModel.getInstance(Constants.FileNames.getClose()).applyFunction(dist);
            } else {
                angle = RobotContainer.drive.getPose().getY() > Constants.Misc.SOURCE_AMP_CUTOFF ?
                DistanceToArmAngleModel.getInstance(Constants.FileNames.getFarAmp()).applyFunction(dist) :
                DistanceToArmAngleModel.getInstance(Constants.FileNames.getFarSource()).applyFunction(dist);
            }
        } else {
            if(RobotContainer.drive.getPose().getX() > Constants.Field.LENGTH - Constants.Misc.CLOSE_FAR_CUTOFF) {
                angle = DistanceToArmAngleModel.getInstance(Constants.FileNames.getClose()).applyFunction(dist);
            } else {
                angle = RobotContainer.drive.getPose().getY() > Constants.Misc.SOURCE_AMP_CUTOFF ?
                DistanceToArmAngleModel.getInstance(Constants.FileNames.getFarAmp()).applyFunction(dist) :
                DistanceToArmAngleModel.getInstance(Constants.FileNames.getFarSource()).applyFunction(dist);
            }
        }
        

        Logger.recordOutput("SetAngle", angle);

       if(Presets.Arm.OVERRIDE_AUTOMATIC_AIM) angle = Presets.Arm.PODIUM_SHOOTER_POS.getRadians();
        if(RobotContainer.subwooferShot) arm.setArmPivot(Presets.Arm.NEUTRAL_POS);
        else arm.setArmPivot(Rotation2d.fromRadians(angle));
    }
    @Override
    public boolean isFinished() {
        Logger.recordOutput("ArmToShoot?", true);
        if(DriverStation.isAutonomous() && Governor.getDesiredRobotState() == RobotState.SHOOT) return Math.abs(angle - arm.getArmPivotAngle().getRadians()) < Presets.Arm.POS_TOLERANCE.getRadians() || !RobotContainer.sensors.getShooterSensor();
        else return Governor.getRobotState() != RobotState.SHOOT_PREP;

    }
}
