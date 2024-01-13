package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveTestCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {

  public static final DriveSubsystem drive = new DriveSubsystem();

  public RobotContainer() {
    SmartDashboard.putData(new SwerveTestCommand());
    configureBindings();
  }

  private void configureBindings() {
    
  }
}
