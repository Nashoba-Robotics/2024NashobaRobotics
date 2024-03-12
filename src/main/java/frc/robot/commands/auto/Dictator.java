package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

public class Dictator extends Command{
    private LoaderSubsystem loader = RobotContainer.loader;

    private boolean shootFlag;
    private Timer shootTimer;

    public Dictator() {
        shootFlag = false;
        shootTimer = new Timer();
    }

    @Override
    public void initialize() {
        shootFlag = false;
    }

    @Override
    public void execute() {

        if(shootFlag && Governor.getRobotState() != RobotState.SHOOT) shootFlag = false;

        switch (Governor.getRobotState()) {
            case NEUTRAL:
                break;
            case TRANSITION:
                break;
            case INTAKE:
                if(RobotContainer.sensors.getShooterSensor()) Governor.setRobotState(RobotState.SHOOT_PREP);
                break;
            case SOURCE:
                // if(loader.getLoaderSensor()) Governor.setRobotState(RobotState.NEUTRAL);
                break;
            case SHOOT_PREP:
                break;
            case SHOOT:
                if(!shootFlag){
                    shootTimer.restart();
                    shootFlag = true;
                }
                if(shootFlag && shootTimer.get() > 0.1
                && !RobotContainer.sensors.getLoaderSensor()
                && !RobotContainer.sensors.getShooterSensor()){
                    Governor.setRobotState(RobotState.INTAKE);
                    shootFlag = false;
                    shootTimer.stop();
                } 
                break;
            case AMP:
               break; 
            case SHUTTLE:
                break;
            default:
                break;
        }
    }
}
