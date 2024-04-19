package frc.robot.commands.setters.units;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class StopAllRollers extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    IntakeSubsystem intake = RobotContainer.intake;
    LoaderSubsystem loader = RobotContainer.loader;
    
    public StopAllRollers(){
        addRequirements(arm, intake, loader);
    }

    @Override
    public void initialize() {
        // if(DriverStation.isAutonomous()) {
        //     arm.setShooterPercent(0);
        // }
        loader.setRollerSpeed(0);
        if(Governor.getDesiredRobotState() != RobotState.INTAKE) intake.setSpeed(0);
    }

    @Override
    public void execute() {
        if(DriverStation.isTeleop() && !Governor.cleanUp) {
            // arm.setShooterPercent(0);
        }
        loader.setRollerSpeed(0);
        if(Governor.getDesiredRobotState() != RobotState.INTAKE) intake.setSpeed(0);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
