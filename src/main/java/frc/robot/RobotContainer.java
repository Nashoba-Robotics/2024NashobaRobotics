package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveTestCommand;
import frc.robot.commands.test.ArmTuneCommand;
import frc.robot.commands.auto.source.ToSource0Command;
import frc.robot.commands.auto.source.ToSource1Command;
import frc.robot.commands.auto.source.ToSource2Command;
import frc.robot.commands.setters.units.arm.ArmToAmp;
import frc.robot.commands.setters.units.arm.ArmToIntake;
import frc.robot.commands.setters.units.arm.ArmToNeutral;
import frc.robot.commands.setters.units.arm.ArmToSource;
import frc.robot.commands.setters.units.loader.LoaderToAmp;
import frc.robot.commands.setters.units.loader.LoaderToIntake;
import frc.robot.commands.setters.units.loader.LoaderToNeutral;
import frc.robot.commands.setters.units.loader.LoaderToSource;
import frc.robot.commands.test.FindLoaderZero;
import frc.robot.commands.test.IntakeTestCommand;
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
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class RobotContainer {
  public static final DriveSubsystem drive = new DriveSubsystem();
  public static final JoystickSubsystem joysticks = new JoystickSubsystem();
  // public static final AprilTagManager aprilTags = new AprilTagManager();
  public static final ArmSubsystem arm = new ArmSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  public static final LoaderSubsystem loader = new LoaderSubsystem();
  
  private static SendableChooser<Command> autoChooser;

  private static Trigger seemlessPath = joysticks.getDriverController().button(1);
  private static Trigger zeroGyro = joysticks.getDriverController().button(2);

  private Trigger incrementSource = joysticks.getOperatorController().button(6);
  private Trigger decrementSorce = joysticks.getOperatorController().button(5);
  private Trigger toSource = joysticks.getOperatorController().button(2);

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
    seemlessPath.onTrue(new OnTheFlytoPathCommand());
    zeroGyro.onTrue(new InstantCommand(()-> drive.setGyro(0)));

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
    // SmartDashboard.putData(new FindLoaderZero(arm));

    
    
    SmartDashboard.putData(new ArmTuneCommand(arm));


      SmartDashboard.putData(new ArmTuneCommand(arm));
      SmartDashboard.putData(new LoaderTuneCommand(loader));

      SmartDashboard.putData("Loader 0", new InstantCommand(()->{
        loader.setPivotRotor(Rotation2d.fromRadians(0));
      }));
      SmartDashboard.putData(new InstantCommand(()->arm.setArmPivotRotor(Rotation2d.fromDegrees(0))));
      SmartDashboard.putData("Zero From Intake", new InstantCommand(()->arm.setArmPivotRotor(Presets.Arm.INTAKE_POS)));

      SmartDashboard.putData(new IntakeTestCommand(intake));

      SmartDashboard.putData(new ArmToNeutral());
      SmartDashboard.putData(new ArmToIntake());
      SmartDashboard.putData(new ArmToAmp());
      SmartDashboard.putData(new ArmToSource());

    // SmartDashboard.putData(new LoaderToNeutral());
    // SmartDashboard.putData(new LoaderToIntake());
    // SmartDashboard.putData(new LoaderToSource());
    // SmartDashboard.putData(new LoaderToAmp());

    // SmartDashboard.putData(new SwerveTestCommand(drive));
  }

  private void configureEvents() {
    // NamedCommands.registerCommand("Name", command);
  }

  private int sourceIndex;

  private void configureSourceBindings() {
    decrementSorce.onTrue(new InstantCommand(() -> {
      sourceIndex+=2;
      sourceIndex %= 3;

      SmartDashboard.putNumber("sourceIndex", sourceIndex);
    }));

    incrementSource.onTrue(new InstantCommand(() -> {
      sourceIndex++;
      sourceIndex %= 3;

      SmartDashboard.putNumber("sourceIndex", sourceIndex);
    }));

    toSource.and(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return sourceIndex == 0;
      }
    }).onTrue(
      new ToSource0Command()
    );

    toSource.and(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return sourceIndex == 1;
      }
    }).onTrue(
      new ToSource1Command()
    );

    toSource.and(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return sourceIndex == 2;
      }
    }).onTrue(
      new ToSource2Command()
    );
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }
}
