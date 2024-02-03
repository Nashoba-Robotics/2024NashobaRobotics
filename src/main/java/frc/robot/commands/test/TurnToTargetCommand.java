package frc.robot.commands.test;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TurnToTargetCommand extends Command{
    ProfiledPIDController controller;
    TrapezoidProfile trapezoidController;
    DriveSubsystem drive;
    double startAngle;
    Timer timer;

    public TurnToTargetCommand(DriveSubsystem drive){
        controller = new ProfiledPIDController(0, 0, 0, new Constraints(2, 4));
        trapezoidController = new TrapezoidProfile(new Constraints(2, 2));
        this.drive = drive;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        startAngle = drive.getGyroAngle().getRadians();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        drive.set(0, 0, 
        //controller.calculate(drive.getGyroAngle().getRadians(), startAngle + 30*Constants.TAU/360));
        trapezoidController.calculate(timer.get(), new State(startAngle, 0), new State(startAngle+Constants.TAU/4, 0)).velocity
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
