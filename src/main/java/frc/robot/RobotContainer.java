package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Governor.RobotState;
import frc.robot.commands.AimToAmpCommand;
import frc.robot.commands.AimToSpeakerCommand;
import frc.robot.commands.auto.source.ToSource0Command;
import frc.robot.commands.auto.source.ToSource1Command;
import frc.robot.commands.auto.source.ToSource2Command;
import frc.robot.commands.setters.groups.ToPuke;
import frc.robot.commands.test.ManualShootCommand;
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

  private static Trigger seemlessPath = joysticks.getDriverController().button(1);
  private static Trigger zeroGyro = joysticks.getDriverController().button(12);

  private Trigger incrementSource = joysticks.getDriverController().button(6);
  private Trigger decrementSorce = joysticks.getDriverController().button(5);
  private Trigger toSource = joysticks.getDriverController().button(7);

  private Trigger groundIntake = joysticks.getDriverController().button(2);
  private Trigger shoot = joysticks.getDriverController().button(8);
  private Trigger neutralMode = joysticks.getDriverController().button(10);

  private Trigger toAmpPrep = joysticks.getDriverController().button(5);
  private Trigger scoreAmp = joysticks.getDriverController().button(3);

  private Trigger puke = joysticks.getDriverController().button(9);
  private Trigger shootPrep = joysticks.getDriverController().button(6);

  private Trigger incrementAngle = joysticks.getOperatorController().button(8); //rt  Shot goes higher
  private Trigger decrementAngle = joysticks.getOperatorController().button(7); //lt  Shot goes lower

  private Trigger increaseSpeed = joysticks.getOperatorController().button(6);  //rb
  private Trigger decreaseSpeed = joysticks.getOperatorController().button(5);  //lb

  private Trigger operatorPrepShoot = joysticks.getOperatorController().button(-1);

  private Trigger deployClimb = joysticks.getOperatorController().button(4);  //x
  private Trigger climb = joysticks.getOperatorController().button(3);  //A

  public static AimToSpeakerCommand aimToSpeakerCommand = new AimToSpeakerCommand(drive, joysticks);

  public static enum NoteState{
    NONE,
    SHOOTER,
    LOADER
  }

  public static NoteState noteState = NoteState.NONE;

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
    zeroGyro.onTrue(new InstantCommand(()-> drive.zeroAngle()));

    groundIntake.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.INTAKE)));
    shoot.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.SHOOT)));

    neutralMode.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.NEUTRAL, true)));

    scoreAmp.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.AMP)));
    toAmpPrep.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.AMP_ADJ)));
    toAmpPrep.onTrue(new AimToAmpCommand(drive, joysticks));
    // toAmpPrep.onTrue(new ToAmpCommand());

    toSource.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.SOURCE))); // TODO: check if we can call onTrue twice and have both commands still work

    puke.onTrue(new ToPuke());
    shootPrep.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.SHOOT_PREP)));

    incrementAngle.onTrue(new InstantCommand(()->Presets.Arm.SPEAKER_OFFSET = Rotation2d.fromDegrees(Presets.Arm.SPEAKER_OFFSET.getDegrees() - 2)));
    decrementAngle.onTrue(new InstantCommand(()->Presets.Arm.SPEAKER_OFFSET = Rotation2d.fromDegrees(Presets.Arm.SPEAKER_OFFSET.getDegrees() + 2)));

    increaseSpeed.onTrue(new InstantCommand(()->Presets.Arm.SPEAKER_SPEED = Rotation2d.fromRadians(Presets.Arm.SPEAKER_SPEED.getRadians() + 10)));
    decreaseSpeed.onTrue(new InstantCommand(()->Presets.Arm.SPEAKER_SPEED = Rotation2d.fromRadians(Presets.Arm.SPEAKER_SPEED.getRadians() - 10)));

    operatorPrepShoot.onTrue(new InstantCommand(()->Governor.setRobotState(RobotState.SHOOT_PREP)));    
    shootPrep.onTrue(new AimToSpeakerCommand(drive, joysticks));
  }

  private void addShuffleBoardData() {
    SmartDashboard.putData(new ManualShootCommand(loader, arm));
  }

  private void configureEvents() {   
    NamedCommands.registerCommand("ShootPrep", new InstantCommand(() -> Governor.setRobotState(RobotState.SHOOT_PREP, true)));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> Governor.setRobotState(RobotState.INTAKE, true)));
    NamedCommands.registerCommand("ShootWait", new SequentialCommandGroup(
      new WaitUntilCommand(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
          return loader.getShooterSensor() && Governor.getRobotState() == RobotState.SHOOT_PREP;
      }
    }).withTimeout(2),
      new InstantCommand(() -> Governor.setRobotState(RobotState.SHOOT, true)),
      new WaitUntilCommand(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return Governor.getRobotState() != RobotState.SHOOT;
        }
      })
    ));
    NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
      new InstantCommand(() -> Governor.setRobotState(RobotState.SHOOT, true)),
      new WaitUntilCommand(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return Governor.getRobotState() != RobotState.SHOOT;
        }
      })
    ));
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
