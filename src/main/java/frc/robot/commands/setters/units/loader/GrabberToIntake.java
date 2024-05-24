package frc.robot.commands.setters.units.loader;

import org.photonvision.estimation.RotTrlTransform3d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

public class GrabberToIntake extends Command {
    private LoaderSubsystem loader = RobotContainer.loader;    

    public 
    GrabberToIntake() {
        addRequirements(loader);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double loaderSpeed = RobotContainer.sensors.getShooterSensor() ? 0 : Presets.Loader.INTAKE_SPEED;
        loader.setRollerSpeed(loaderSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // if(!interrupted) loader.setRollerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(loader.getLoaderPivotAngle().getRadians() - Presets.Loader.INTAKE_SPEED.getRadians()) < Presets.Loader.SPEED_TOLERANCE.getRadians();
        // return loader.getShooterSensor();
        return true;
    }

}
