package frc.robot.commands.auto.remaps;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;

public class P6CheckTo7 extends Command {
    
    @Override
    public void initialize() {
        if(!RobotContainer.sensors.getLoaderSensor() && !RobotContainer.sensors.getShooterSensor()) {
            CommandScheduler.getInstance().schedule(AutoBuilder.buildAuto("NoP6To7"));
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
