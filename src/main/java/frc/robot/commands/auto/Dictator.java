package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.lib.util.DistanceToArmAngleModel;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

public class Dictator extends Command{
    private LoaderSubsystem loader = RobotContainer.loader;
    private DriveSubsystem drive = RobotContainer.drive;

    private boolean shootFlag;
    private Timer shootTimer;

    private boolean shuttleFlag;
    private Timer shuttleTimer;

    public Dictator() {
        shootFlag = false;
        shootTimer = new Timer();

        shuttleFlag = false;
        shuttleTimer = new Timer();
    }

    @Override
    public void initialize() {
        shootFlag = false;
        shuttleFlag = false;
    }

    @Override
    public void execute() {

        if(shootFlag && Governor.getDesiredRobotState() != RobotState.SHOOT) shootFlag = false;
        if(shuttleFlag && Governor.getDesiredRobotState() != RobotState.SHUTTLE) shuttleFlag = false;

        switch (Governor.getRobotState()) {
            case NEUTRAL:
                break;
            case TRANSITION:
                break;
            case INTAKE:
                // if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    if(RobotContainer.sensors.getShooterSensor()) Governor.setRobotState(RobotState.NEUTRAL);
                // } else {
                //     if(RobotContainer.sensors.getShooterSensor()
                //     && RobotContainer.drive.getPose().getX() < Constants.Field.LENGTH - 4) Governor.setRobotState(RobotState.NEUTRAL);
                // }
                break;
            case SOURCE:
                // if(loader.getLoaderSensor()) Governor.setRobotState(RobotState.NEUTRAL);
                break;
            case SHOOT_PREP:
                break;
            case SHOOT:
                if(!shootFlag
                && !RobotContainer.sensors.getLoaderSensor()
                && !RobotContainer.sensors.getShooterSensor()){
                    shootTimer.restart();
                    shootFlag = true;
                }
                if(shootFlag && shootTimer.get() > 0.05
                ){

                    if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                        if(RobotContainer.drive.getPose().getX() < Constants.Misc.CLOSE_FAR_CUTOFF) {
                            RobotContainer.lastModelForShot = Constants.FileNames.getClose();
                            DistanceToArmAngleModel.getInstance(Constants.FileNames.getClose()).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                        } else {
                            if(drive.getPose().getTranslation().getY() < Constants.Misc.SOURCE_AMP_CUTOFF) {
                            RobotContainer.lastModelForShot = Constants.FileNames.getFarSource();
                            DistanceToArmAngleModel.getInstance(Constants.FileNames.getFarSource()).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                            } else {
                                RobotContainer.lastModelForShot = Constants.FileNames.getFarAmp();
                                DistanceToArmAngleModel.getInstance(Constants.FileNames.getFarAmp()).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                            }
                        }
                    } else {
                        if(RobotContainer.drive.getPose().getX() > Constants.Field.LENGTH - Constants.Misc.CLOSE_FAR_CUTOFF) {
                            RobotContainer.lastModelForShot = Constants.FileNames.getClose();
                            DistanceToArmAngleModel.getInstance(Constants.FileNames.getClose()).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                        } else {
                            if(drive.getPose().getTranslation().getY() < Constants.Misc.SOURCE_AMP_CUTOFF) {
                            RobotContainer.lastModelForShot = Constants.FileNames.getFarSource();
                            DistanceToArmAngleModel.getInstance(Constants.FileNames.getFarSource()).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                            } else {
                                RobotContainer.lastModelForShot = Constants.FileNames.getFarAmp();
                                DistanceToArmAngleModel.getInstance(Constants.FileNames.getFarAmp()).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                            }
                        }
                    }

                    if(RobotContainer.disruptFlag) Governor.setRobotState(RobotState.INTAKE);
                    else Governor.setRobotState(RobotState.NEUTRAL);
                    shootFlag = false;
                    shootTimer.stop();
                } 
                break;
            case AMP:
               break; 
            case SHUTTLE:
                if(!shuttleFlag
                && !RobotContainer.sensors.getLoaderSensor()
                && !RobotContainer.sensors.getShooterSensor()){
                    shuttleTimer.restart();
                    shuttleFlag = true;
                }
                if(shuttleFlag && shuttleTimer.get() > 0.2
                ){
                    Governor.setRobotState(RobotState.INTAKE);
                    shuttleFlag = false;
                    shuttleTimer.stop();
                } 
                break;
            default:
                break;
        }
    }
}
