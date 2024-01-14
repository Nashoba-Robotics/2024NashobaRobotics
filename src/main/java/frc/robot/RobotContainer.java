package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveTestCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class RobotContainer {

  public static final DriveSubsystem drive = new DriveSubsystem();
  public static final JoystickSubsystem joysticks = new JoystickSubsystem();

  public RobotContainer() {
    SmartDashboard.putData(new SwerveTestCommand());
    SmartDashboard.putData(new DriveCommand());
    configureBindings();
  }

  private void configureBindings() {
    
  }
}
