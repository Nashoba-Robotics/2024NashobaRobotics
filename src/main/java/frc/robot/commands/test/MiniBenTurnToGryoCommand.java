package frc.robot.commands.test;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class MiniBenTurnToGryoCommand extends Command{
    public DriveSubsystem drive = RobotContainer.drive;
    public ProfiledPIDController controller;
    public MiniBenTurnToGryoCommand(){
        controller = new ProfiledPIDController(0, 0, 0, null);//TODO add PID and trapezoidprofile values
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // controller.setConstraints(0.5, 0.5);
        controller.setGoal(0);
        controller.setGoal(0);
        // controller.set
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
