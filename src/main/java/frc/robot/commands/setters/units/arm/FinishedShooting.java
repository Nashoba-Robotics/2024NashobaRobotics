package frc.robot.commands.setters.units.arm;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class FinishedShooting extends Command {
    
    private ArmSubsystem arm = RobotContainer.arm;

    Timer t;
    boolean flag;

    public FinishedShooting() {
        addRequirements(arm);
        t = new Timer();
        flag = false;
    }

    @Override
    public void initialize() {
        flag = false;
    }

    @Override
    public void execute() {
        if(arm.getShooterSpeed().getRadians() < Presets.Arm.SPEAKER_SPEED.getRadians() - 30 && !flag) {
            flag = true;
            t.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return t.get() > 0.1 && !RobotContainer.loader.getLoaderSensor() && !RobotContainer.loader.getShooterSensor();
    }

}
