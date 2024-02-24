package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveState;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class President extends Command {
    LoaderSubsystem loader = RobotContainer.loader;
    DriveSubsystem drive = RobotContainer.drive;

    private boolean shootFlag;
    private Timer shootTimer;

    private boolean ampFlag;
    private Timer ampTimer;

    private boolean queueFlag;
    private Timer queueTimer;

    public President() {
        shootFlag = false;
        shootTimer = new Timer();
    
        ampFlag = false;
        ampTimer = new Timer();

        queueFlag = false;
        queueTimer = new Timer();
    }

    @Override
    public void initialize() {
        shootFlag = false;
        ampFlag = false;
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
                break;
            case TRANSITION:
                //TODO:
                break;
            case INTAKE:
                if(loader.getShooterSensor()) Governor.setRobotState(RobotState.NEUTRAL);
                break;
            case SOURCE:
                if(loader.getLoaderSensor()) Governor.setRobotState(RobotState.NEUTRAL);
                break;
            case SHOOT_PREP:
                // drive.state = DriveState.AIM_TO_SPEAKER;
                break;
            case SHOOT:
                if(!shootFlag){
                    shootTimer.restart();
                    shootFlag = true;
                }
                if(shootTimer.get() > 0.1
                && !RobotContainer.loader.getLoaderSensor()
                && !RobotContainer.loader.getShooterSensor()){
                    Governor.setRobotState(RobotState.NEUTRAL);
                    shootFlag = false;
                    shootTimer.stop();
                } 

                //TODO: When odometry is in a certain range, go to shoot prep
                break;
            case AMP_ADJ:
                // drive.state = DriveState.AIM_TO_AMP;
                break;
            case AMP:
                if(!ampFlag){
                        ampTimer.restart();
                        ampFlag = true;
                    }
                    if(ampTimer.get() > 0.1
                    && !RobotContainer.loader.getLoaderSensor()
                    && !RobotContainer.loader.getShooterSensor()){
                        Governor.setRobotState(RobotState.NEUTRAL);
                        ampFlag = false;
                        ampTimer.stop();
                    } 
               break; 
            default:
                break;
        }
    }

    
}