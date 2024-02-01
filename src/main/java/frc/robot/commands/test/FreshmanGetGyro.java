package frc.robot.commands.test;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class FreshmanGetGyro extends Command{
    public DriveSubsystem drive = RobotContainer.drive;
    public PIDController controller;
    public TrapezoidProfile profile;

    public TrapezoidProfile.State goal = new TrapezoidProfile.State();
    public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    Timer timer = new Timer();
    
    public FreshmanGetGyro(){
        controller = new PIDController(3.8, 0, 0.4);
        profile = new TrapezoidProfile(new Constraints(3, 4));
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        goal = new TrapezoidProfile.State(0,0);
        timer.reset();
        timer.start();
        setpoint = new TrapezoidProfile.State(drive.getGyroAngle().getRadians(), 0);
    }

    @Override
    public void execute() {
        double currTime = timer.get();

        TrapezoidProfile.State profileOutput = profile.calculate(currTime, setpoint, goal);
        double PIDoutput = controller.calculate(drive.getGyroAngle().getRadians(), profileOutput.position);
        drive.set(new ChassisSpeeds(0, 0, profileOutput.velocity + PIDoutput));
        Logger.recordOutput("RobotPosition", drive.getGyroAngle());
        Logger.recordOutput("goal", profileOutput.position);
        Logger.recordOutput("desired velocity", profileOutput.velocity);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
