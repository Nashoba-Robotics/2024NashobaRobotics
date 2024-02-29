package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Governor;
import frc.robot.RobotContainer;

public class Dictator extends Command{
    @Override
    public void execute() {
        switch (Governor.getRobotState()) {
            case NEUTRAL:
                // drive.state = DriveState.DRIVER;
                // if(loader.getShooterSensor()) Governor.setRobotState(RobotState.SHOOT_PREP);
                break;
            case TRANSITION:
                //TODO:
                break;
            case INTAKE:
                // if(loader.getShooterSensor()) Governor.setRobotState(RobotState.NEUTRAL);
                break;
            case SOURCE:
                // if(loader.getLoaderSensor()) Governor.setRobotState(RobotState.NEUTRAL);
                break;
            case SHOOT_PREP:
                // if(loader.getLoaderSensor)
                // // drive.state = DriveState.AIM_TO_SPEAKER;
                // if(Math.abs(joysticks.getRightJoystickValues().x) <= 0.03
                // && !CommandScheduler.getInstance().isScheduled(aimToSpeakerCommand))
                // CommandScheduler.getInstance().schedule(aimToSpeakerCommand);
                break;
            case SHOOT:
                // if(!shootFlag){
                //     shootTimer.restart();
                //     shootFlag = true;
                // }
                // if(shootFlag && shootTimer.get() > 0.1
                // && !RobotContainer.loader.getLoaderSensor()
                // && !RobotContainer.loader.getShooterSensor()){
                //     Governor.setRobotState(RobotState.NEUTRAL);
                //     shootFlag = false;
                //     shootTimer.stop();
                // } 

                //TODO: When odometry is in a certain range, go to shoot prep
                break;
            case AMP:
                // if(!ampFlag){
                //         ampTimer.restart();
                //         ampFlag = true;
                // }
                // if(ampFlag && ampTimer.get() > 1
                // && !RobotContainer.loader.getLoaderSensor()
                // && !RobotContainer.loader.getShooterSensor()){
                //     Governor.setRobotState(RobotState.NEUTRAL);
                //     ampFlag = false;
                //     ampTimer.stop();
                // } 
               break; 
            default:
                break;
        }
    }
}
