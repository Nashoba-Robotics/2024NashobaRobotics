package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TurnTestCommand extends Command {
    
    private DriveSubsystem drive;

    public TurnTestCommand(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Angle", 0);
    }

    @Override
    public void execute() {
        for(int i = 0; i < 4; i++)
        drive.setState(i, new SwerveModuleState(0.01, Rotation2d.fromDegrees(SmartDashboard.getNumber("Angle", 0))));
    }

}
