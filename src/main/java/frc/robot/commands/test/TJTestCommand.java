package frc.robot.commands.test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TJTestCommand extends Command{
    
    private DriveSubsystem drive = RobotContainer.drive;
    private double voltage;

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Volts", 0);

        drive.setGyro(Constants.TAU/2);
        // drive.set(new ChassisSpeeds(0, 0, 0));;
        drive.setTurnStates(0);

        voltage = 0;
    }

    @Override
    public void execute() {
        // double v = SmartDashboard.getNumber("Volts", 0);

        drive.setVoltageStates(voltage);
        drive.setTurnStates(0);

        SmartDashboard.putNumber("Voltage", voltage);

        // drive.setTurnStates(v);


        voltage += 0.004;   //10-20ms loop time
    }

    @Override
    public void end(boolean interrupted) {
        drive.setVoltageStates(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
