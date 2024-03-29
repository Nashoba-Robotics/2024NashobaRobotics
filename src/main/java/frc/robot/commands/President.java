package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.Presets;
import frc.robot.lib.util.DistanceToArmAngleModel;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

public class President extends Command {
    LoaderSubsystem loader = RobotContainer.loader;
    DriveSubsystem drive = RobotContainer.drive;
    JoystickSubsystem joysticks = RobotContainer.joysticks;

    private boolean shootFlag;
    private Timer shootTimer;

    private boolean ampFlag;
    private boolean ampSensorFlag;
    private Timer ampTimer;

    private boolean shuttleFlag;
    private Timer shuttleTimer;

    private boolean queueFlag;
    private Timer queueTimer;

    private AimToSpeakerCommand aimToSpeakerCommand;

    public President() {
        shootFlag = false;
        shootTimer = new Timer();
    
        ampFlag = false;
        ampTimer = new Timer();

        queueFlag = false;
        queueTimer = new Timer();

        shuttleFlag = false;
        shuttleTimer = new Timer();

        aimToSpeakerCommand = RobotContainer.aimToSpeakerCommand;
    }

    @Override
    public void initialize() {
        shootFlag = false;
        ampFlag = false;
        shuttleFlag = false;
    }

    @Override
    public void execute() {

        if(shootFlag && Governor.getRobotState() != RobotState.SHOOT) shootFlag = false;

        if(Governor.getQueuedState() != RobotState.UNKNOWN) {
            if(!queueFlag) {
                queueTimer.restart();
                queueFlag = true;
            }

            if(queueTimer.get() > 5.0) Governor.setQueuedState(RobotState.UNKNOWN);

            if(Governor.getRobotState() != RobotState.TRANSITION) {
                Governor.setRobotState(Governor.getQueuedState());
                Governor.setQueuedState(RobotState.UNKNOWN);
            }
        } else {
            queueFlag = false;
        }

        switch (Governor.getRobotState()) {
            case NEUTRAL:
                // drive.state = DriveState.DRIVER;
                // if(loader.getShooterSensor()) Governor.setRobotState(RobotState.SHOOT_PREP);
                // if(RobotContainer.sensors.getShooterSensor() && Governor.getLastRobotState()==RobotState.INTAKE) 
                //     CommandScheduler.getInstance().schedule(new InstantCommand(()->RobotContainer.intake.setSpeed(-0.1), RobotContainer.intake));
                break;
            case TRANSITION:
                //TODO:
                break;
            case INTAKE:
                if(RobotContainer.sensors.getShooterSensor()) Governor.setRobotState(RobotState.NEUTRAL);
                break;
            case SOURCE:
                if(RobotContainer.sensors.getShooterSensor()) Governor.setRobotState(RobotState.NEUTRAL);
                break;
            case SHOOT_PREP:
                // drive.state = DriveState.AIM_TO_SPEAKER;
                // if(Math.abs(joysticks.getRightJoystickValues().x) <= 0.03
                // && !CommandScheduler.getInstance().isScheduled(aimToSpeakerCommand))
                // CommandScheduler.getInstance().schedule(aimToSpeakerCommand);
                break;
            case SHOOT:
                if(!shootFlag){
                    shootTimer.restart();
                    shootFlag = true;
                }
                if(shootFlag && shootTimer.get() > 0.1
                && !RobotContainer.sensors.getLoaderSensor()
                && !RobotContainer.sensors.getShooterSensor()){

                    if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                        if(RobotContainer.drive.getPose().getX() < 3.15) {
                            RobotContainer.lastModelForShot = Constants.FileNames.ARM_ANGLE_CLOSE;
                            DistanceToArmAngleModel.getInstance(Constants.FileNames.ARM_ANGLE_CLOSE).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                        } else {
                            if(drive.getPose().getTranslation().getY() < 4.1) {
                            RobotContainer.lastModelForShot = Constants.FileNames.ARM_ANGLE_FAR_SOURCE;
                            DistanceToArmAngleModel.getInstance(Constants.FileNames.ARM_ANGLE_FAR_SOURCE).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                            } else {
                                RobotContainer.lastModelForShot = Constants.FileNames.ARM_ANGLE_FAR_AMP;
                                DistanceToArmAngleModel.getInstance(Constants.FileNames.ARM_ANGLE_FAR_AMP).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                            }
                        }
                    } else {
                        if(RobotContainer.drive.getPose().getX() > Constants.Field.LENGTH - 3.15) {
                            RobotContainer.lastModelForShot = Constants.FileNames.ARM_ANGLE_CLOSE;
                            DistanceToArmAngleModel.getInstance(Constants.FileNames.ARM_ANGLE_CLOSE).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                        } else {
                            if(drive.getPose().getTranslation().getY() < 4.1) {
                            RobotContainer.lastModelForShot = Constants.FileNames.ARM_ANGLE_FAR_SOURCE;
                            DistanceToArmAngleModel.getInstance(Constants.FileNames.ARM_ANGLE_FAR_SOURCE).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                            } else {
                                RobotContainer.lastModelForShot = Constants.FileNames.ARM_ANGLE_FAR_AMP;
                                DistanceToArmAngleModel.getInstance(Constants.FileNames.ARM_ANGLE_FAR_AMP).lastDistanceToShoot = drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
                            }
                        }
                    }



                    
                    Governor.setRobotState(RobotState.NEUTRAL);
                    shootFlag = false;
                    shootTimer.stop();
                } 

                //TODO: When odometry is in a certain range, go to shoot prep
                break;
            case AMP:
                if(!ampFlag){
                        ampTimer.restart();
                        ampFlag = true;
                }
                if(!ampSensorFlag && RobotContainer.sensors.getLoaderSensor()){
                    ampSensorFlag = true;
                }
                if(ampFlag && !RobotContainer.sensors.getLoaderSensor() && ampSensorFlag){
                    Governor.setRobotState(RobotState.NEUTRAL, false);
                    ampFlag = false;
                    ampSensorFlag = false;
                    ampTimer.stop();
                } 
               break; 
            case SHUTTLE:
                if(!shuttleFlag){
                    shuttleTimer.restart();
                    shuttleFlag = true;
                }
                if(shuttleFlag 
                && !RobotContainer.sensors.getShooterSensor() 
                && shuttleTimer.get()>0.1){
                    shuttleFlag = false;
                    shuttleTimer.stop();
                    Governor.setRobotState(RobotState.NEUTRAL);
                }
                break;
            default:
                break;
        }
    }

    
}
