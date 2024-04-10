package frc.robot.commands.test;


import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.Constants.Robot;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

public class SourceShuttleTest extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    LoaderSubsystem loader = RobotContainer.loader;
    SensorManager sensors = RobotContainer.sensors;

    public SourceShuttleTest(){
        addRequirements(arm, loader);
    }

    @Override
    public void initialize() {
        arm.setShooterSpeed(Rotation2d.fromRadians(300));
        arm.configMotionMagic(0.95, 3);
    }

    @Override
    public void execute() {
        if(sensors.getShooterSensor() || sensors.getIntakeSensor()){
            arm.setArmPivot(Presets.Arm.HIGH_SHUTTLE_POS);
            if(Math.abs(arm.getArmPivotAngle().getRadians() - Presets.Arm.HIGH_SHUTTLE_POS.getRadians()) <= .5){
                loader.setRollerSpeed(Presets.Loader.SHUTTLE_SPEED);
            }
            else loader.setRollerSpeed(0);
        }
        else{
            loader.setRollerSpeed(sensors.getShooterSensor() ? 0 : Presets.Loader.SOURCE_SPEED);
            if(Math.abs(arm.getArmPivotAngle().getRadians() - Presets.Arm.SOURCE_POS.getRadians()) <= Presets.Arm.POS_TOLERANCE.getRadians()){

            }
            else{
                arm.setArmPivot(Presets.Arm.SOURCE_POS);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.configMotionMagic(Constants.Arm.PIVOT_MOTION_MAGIC_CRUISE_VELOCITY, Constants.Arm.PIVOT_MOTION_MAGIC_ACCELERATION);
    }
}
