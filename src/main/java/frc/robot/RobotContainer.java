package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveTestCommand;
import frc.robot.commands.test.TJTestCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class RobotContainer {

  public static final DriveSubsystem drive = new DriveSubsystem();
  public static final JoystickSubsystem joysticks = new JoystickSubsystem();

  private static SendableChooser<Command> autoChooser;

  public RobotContainer() {
    addShuffleBoardData();
    configureBindings();
    configureEvents();

    // Logging callback for target robot pose
      PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
          Logger.recordOutput("TargetPose", pose);
      });

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    
  }

  private void addShuffleBoardData() {
    SmartDashboard.putData(new SwerveTestCommand());
    SmartDashboard.putData(new DriveCommand());
  }

  private void configureEvents() {
    // NamedCommands.registerCommand("Name", command);
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }
}
