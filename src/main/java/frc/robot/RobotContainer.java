package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.test.AimToSpeakerCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveTestCommand;
import frc.robot.commands.W0ShootCommand;
import frc.robot.commands.test.AimToSpeakerCommand;
import frc.robot.commands.test.ArmTuneCommand;
import frc.robot.commands.auto.source.ToSource0Command;
import frc.robot.commands.auto.source.ToSource1Command;
import frc.robot.commands.auto.source.ToSource2Command;
import frc.robot.commands.setters.groups.ToAmp;
import frc.robot.commands.setters.groups.ToIntake;
import frc.robot.commands.setters.groups.ToIntakeAdj;
import frc.robot.commands.setters.groups.ToNeutral;
import frc.robot.commands.setters.groups.ToPuke;
import frc.robot.commands.setters.groups.ToShoot;
import frc.robot.commands.setters.groups.ToSource;
import frc.robot.commands.setters.groups.ToSourceAdj;
import frc.robot.commands.setters.units.StopAllRollers;
import frc.robot.commands.setters.units.arm.ArmToNeutral;
import frc.robot.commands.setters.units.arm.ArmToShoot;
import frc.robot.commands.setters.units.arm.ShooterToShoot;
// import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.test.LoaderTuneCommand;
import frc.robot.commands.test.OnTheFlytoPathCommand;
import frc.robot.commands.test.ShooterTuneCommand;
import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class RobotContainer {
  public static final DriveSubsystem drive = new DriveSubsystem();
  public static final JoystickSubsystem joysticks = new JoystickSubsystem();
  public static final AprilTagManager aprilTags = new AprilTagManager();
  public static final ArmSubsystem arm = new ArmSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  public static final LoaderSubsystem loader = new LoaderSubsystem();
  
  private static SendableChooser<Command> autoChooser;

  // private static Trigger seemlessPath = joysticks.getDriverController().button(1);
  private static Trigger zeroGyro = joysticks.getDriverController().button(12);

  // private Trigger incrementSource = joysticks.getDriverController().button(6);
  // private Trigger decrementSorce = joysticks.getDriverController().button(5);
  private Trigger startShooter = joysticks.getDriverController().button(6);
  private Trigger toSource = joysticks.getDriverController().button(7);
  private Trigger aimToSpeaker = joysticks.getDriverController().button(5);

  private Trigger puke = joysticks.getDriverController().button(9);

  private Trigger groundIntake = joysticks.getDriverController().button(2);
  private Trigger shoot = joysticks.getDriverController().button(8);
  private Trigger neutralMode = joysticks.getDriverController().button(10);


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
    zeroGyro.onTrue(new InstantCommand(()-> drive.setZero()));

    groundIntake.onTrue(new ToIntake());
    // shoot.onTrue(new ToShoot());
    // shoot.onTrue(new W0ShootCommand());
    shoot.onTrue(new ToShoot());
    neutralMode.onTrue(new ToNeutral());
    toSource.onTrue(new ToSource());
    startShooter.onTrue(
      new ParallelCommandGroup(
        new InstantCommand(()-> RobotContainer.arm.setShooterSpeed(Rotation2d.fromRadians(350))),
        new AimToSpeakerCommand(drive, joysticks),
        new ArmToShoot()
    ));

    puke.onTrue(new ToPuke());
    aimToSpeaker.toggleOnTrue(new AimToSpeakerCommand(drive, joysticks));
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
      SmartDashboard.putData(new LoaderTuneCommand(loader));

      SmartDashboard.putData("Loader 0", new InstantCommand(()->{
        loader.setPivotRotor(Rotation2d.fromRadians(0));
      }));
      // SmartDashboard.putData(new InstantCommand(()->arm.setArmPivotRotor(Rotation2d.fromDegrees(0))));
      SmartDashboard.putData("Zero From Intake", new InstantCommand(()->arm.setArmPivotRotor(Presets.Arm.INTAKE_POS)));

      // SmartDashboard.putData(new IntakeTestCommand(intake));

      // SmartDashboard.putData(new ShooterTuneCommand(arm));

      // SmartDashboard.putData(new ArmToNeutral());
      // SmartDashboard.putData(new ArmToIntake());
      // SmartDashboard.putData(new ArmToAmp());
      // SmartDashboard.putData(new ArmToSource());

    // SmartDashboard.putData(new LoaderToNeutral());
    // SmartDashboard.putData(new LoaderToIntake());
    // SmartDashboard.putData(new LoaderToSource());
    // SmartDashboard.putData(new LoaderToAmp());

    // SmartDashboard.putData(new SwerveTestCommand(drive));

    // SmartDashboard.putData(new ToNeutral());
    // SmartDashboard.putData(new ToSourceAdj());
    // SmartDashboard.putData(new ToIntakeAdj());
    // SmartDashboard.putData(new ToIntake());
    // // SmartDashboard.putData(new ToSubwooferShoot());
    // SmartDashboard.putData(new ToShoot());
    SmartDashboard.putData(new ArmToShoot());
    // SmartDashboard.putData(new ToSource());

    // SmartDashboard.putData(new AimToSpeakerCommand());
    SmartDashboard.putData(new InstantCommand(() -> {
      intake.setSpeed(Presets.Intake.INTAKE_SPEED);
        loader.setRollerSpeed(-0.7);
        arm.setShooterPercent(0.2);
      }, intake, loader, arm));
  }

  private void configureEvents() {
    NamedCommands.registerCommand("StartShooter", new ShooterToShoot());
    NamedCommands.registerCommand("Intake", new ToIntake());

    NamedCommands.registerCommand("ShootCommand", new SequentialCommandGroup(
      new ArmToShoot(),
      new ToShoot(),
      new ArmToNeutral()
    ));

  }

  private int sourceIndex;

  private void configureSourceBindings() {
    // decrementSorce.onTrue(new InstantCommand(() -> {
    //   sourceIndex+=2;
    //   sourceIndex %= 3;

    //   SmartDashboard.putNumber("sourceIndex", sourceIndex);
    // }));

    // incrementSource.onTrue(new InstantCommand(() -> {
    //   sourceIndex++;
    //   sourceIndex %= 3;

    //   SmartDashboard.putNumber("sourceIndex", sourceIndex);
    // }));

    // toSource.and(new BooleanSupplier() {
    //   @Override
    //   public boolean getAsBoolean() {
    //     return sourceIndex == 0;
    //   }
    // }).onTrue(
    //   new ToSource0Command()
    // );

    // toSource.and(new BooleanSupplier() {
    //   @Override
    //   public boolean getAsBoolean() {
    //     return sourceIndex == 1;
    //   }
    // }).onTrue(
    //   new ToSource1Command()
    // );

    // toSource.and(new BooleanSupplier() {
    //   @Override
    //   public boolean getAsBoolean() {
    //     return sourceIndex == 2;
    //   }
    // }).onTrue(
    //   new ToSource2Command()
    // );
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }
}
