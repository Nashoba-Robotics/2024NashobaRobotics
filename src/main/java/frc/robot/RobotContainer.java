package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveTestCommand;
import frc.robot.commands.test.LoaderTuneCommand;
import frc.robot.commands.test.OnTheFlyTestCommand;
import frc.robot.commands.test.OnTheFlytoPathCommand;
import frc.robot.commands.test.ResetOdometryCommand;
import frc.robot.commands.test.ResetOdometryVision;
import frc.robot.commands.test.SDFinder;
import frc.robot.commands.test.TurnToTargetCommand;
import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class RobotContainer {

  public static final DriveSubsystem drive = new DriveSubsystem();
  public static final JoystickSubsystem joysticks = new JoystickSubsystem();
  public static final AprilTagManager aprilTags = new AprilTagManager();
  public static final ArmSubsystem arm = new ArmSubsystem();

  private static SendableChooser<Command> autoChooser;

  private static Trigger seemlessPath = joysticks.getLeftJoystick().button(1);

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
    // seemlessPath.onTrue(new OnTheFlytoPathCommand());
  }

  private void addShuffleBoardData() {
    // SmartDashboard.putData(new SwerveTestCommand());
    // SmartDashboard.putData(new DriveCommand());
    // SmartDashboard.putData(new ResetOdometryCommand(drive));
    // SmartDashboard.putData(new OnTheFlyTestCommand());
    // SmartDashboard.putData(new OnTheFlytoPathCommand());
    // SmartDashboard.putData(new ResetOdometryVision());

    // SmartDashboard.putData(new SDFinder());
    // SmartDashboard.putData(new OnTheFlytoPathCommand());
    // SmartDashboard.putData(new TurnToTargetCommand(drive));

    // SmartDashboard.putData(new LoaderTuneCommand(arm));
    // SmartDashboard.putData(new InstantCommand(()->{
    //   arm.setLoaderPivotRotor(Rotation2d.fromRadians(0));
    // }));
  }

  private void configureEvents() {
    // NamedCommands.registerCommand("Name", command);
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }
}
