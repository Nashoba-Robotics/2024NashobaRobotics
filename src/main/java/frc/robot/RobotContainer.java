package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Governor.RobotState;
import frc.robot.commands.AimToSpeakerCommand;
import frc.robot.commands.AimToStation;
import frc.robot.commands.ToggleCleanUpCommand;
import frc.robot.commands.auto.amp.ToAmpCommand;
import frc.robot.commands.auto.remaps.P3Check;
import frc.robot.commands.auto.remaps.P4Check;
import frc.robot.commands.auto.source.ToSource0Command;
import frc.robot.commands.auto.source.ToSource1Command;
import frc.robot.commands.auto.source.ToSource2Command;
import frc.robot.commands.setters.groups.ToPuke;
import frc.robot.commands.setters.units.loader.GrabberToShoot;
import frc.robot.commands.test.ClimberTestCommand;
import frc.robot.commands.test.ClimberTuneCommand;
import frc.robot.commands.test.ManualShootCommand;
import frc.robot.commands.test.TestServoCommand;
import frc.robot.lib.util.DistanceToArmAngleModel;
import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberSubsytem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.leds.LEDManager;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.sensors.SensorManager;

public class RobotContainer {

  public static final DriveSubsystem drive = new DriveSubsystem();
  public static final JoystickSubsystem joysticks = new JoystickSubsystem();
  private static final AprilTagManager aprilTags = new AprilTagManager();
  public static final ArmSubsystem arm = new ArmSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  public static final LoaderSubsystem loader = new LoaderSubsystem();
  public static final LEDManager leds = new LEDManager();
  public static final ClimberSubsytem climber = new ClimberSubsytem();
  public static final SensorManager sensors = new SensorManager();

  public static String lastModelForShot = Constants.FileNames.ARM_ANGLE_CLOSE;
  
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

  private Trigger highShuttle = joysticks.getDriverController().button(4);
  private Trigger lowShuttle = joysticks.getDriverController().button(1);
  public static ToShuttlePrep highShuttlePrep = new ToShuttlePrep(true);
  public static ToShuttlePrep lowShuttlePrep = new ToShuttlePrep(false);
  private boolean lowButtonPressed = false;
  private boolean highButtonPressed = false;

  private Trigger cleanUpMode = joysticks.getDriverController().button(11);

  private Trigger increaseSpeed = joysticks.getOperatorController().button(6);  //rb
  private Trigger decreaseSpeed = joysticks.getOperatorController().button(5);  //lb

  private boolean aimOverrideTriggered = false;
  // private Trigger armAimOverride = joysticks.getOperatorController().button(-1).debounce(0.1);
  private Trigger shootOveride = joysticks.getOperatorController().button(8);
  //Drive override -> B
  // Arm override -> Y

  // private Trigger deployClimb = joysticks.getOperatorController().button(0);
  // private Trigger climb = joysticks.getOperatorController().button(0);

  private Trigger aimedToHigh = joysticks.getOperatorController().button(4); //X
  private Trigger aimedToLow = joysticks.getOperatorController().button(2); //B
  private Trigger aimedJustRight = joysticks.getOperatorController().button(3); //A

  private Trigger prep90 = joysticks.getOperatorController().button(10);


  // private Trigger resetOdometryFromCamera = joysticks.getDriverController.button(11);

  public static AimToSpeakerCommand aimToSpeakerCommand = new AimToSpeakerCommand(drive, joysticks);

  public static enum NoteState{
    NONE,
    SHOOTER,
    LOADER
  }

  public static NoteState noteState = NoteState.NONE;

  public static boolean odometryFlag = false;

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

    scoreAmp.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.AMP, true)));
    toAmpPrep.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.AMP_ADJ)));
    toAmpPrep.onTrue(new SequentialCommandGroup(
      new ToAmpCommand()
      // new InstantCommand(() -> Governor.setRobotState(RobotState.AMP))
    ).until(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
          return joysticks.getRightJoystickValues().x > 0.2 || (Governor.getRobotState() != RobotState.AMP_ADJ && Governor.getRobotState() != RobotState.TRANSITION);
      }
    })
    );

    toSource.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.SOURCE))); // TODO: check if we can call onTrue twice and have both commands still work

    puke.onTrue(new ToPuke());
    shootPrep.onTrue(new InstantCommand(() -> Governor.setRobotState(RobotState.SHOOT_PREP)));
    shootPrep.onTrue(new AimToSpeakerCommand(drive, joysticks));

    // highShuttle.onTrue(new InstantCommand(()->Governor.setRobotState(RobotState.SHUTTLE)));
    // lowShuttle.onTrue(new InstantCommand(()->Governor.setRobotState(RobotState.SHUTTLE_ADJ)));
    lowShuttle.onTrue(new AimToStation(drive, joysticks));

    increaseSpeed.onTrue(new InstantCommand(()->Presets.Arm.SPEAKER_SPEED = Rotation2d.fromRadians(Presets.Arm.SPEAKER_SPEED.getRadians() + 10)));
    decreaseSpeed.onTrue(new InstantCommand(()->Presets.Arm.SPEAKER_SPEED = Rotation2d.fromRadians(Presets.Arm.SPEAKER_SPEED.getRadians() - 10)));

    //TODO: Test this.
    // armAimOverride.onTrue(new InstantCommand(()->{Presets.Arm.OVERRIDE_AUTOMATIC_AIM = true; aimOverrideTriggered = true;})).and(new BooleanSupplier() {
    //   @Override
    //   public boolean getAsBoolean() {
    //       return !Presets.Arm.OVERRIDE_AUTOMATIC_AIM && !aimOverrideTriggered;
    //   } 
    // });
    // armAimOverride.onTrue(new InstantCommand(()->{Presets.Arm.OVERRIDE_AUTOMATIC_AIM = false; aimOverrideTriggered = true;})).and(new BooleanSupplier() {
    //   @Override
    //   public boolean getAsBoolean() {
    //       return Presets.Arm.OVERRIDE_AUTOMATIC_AIM && !aimOverrideTriggered;
    //   } 
    // });
    // armAimOverride.onFalse(new InstantCommand(()->aimOverrideTriggered = false));
    shootOveride.onTrue(new GrabberToShoot());

    aimedToHigh.onTrue(new InstantCommand(() -> {
      DistanceToArmAngleModel instance = DistanceToArmAngleModel.getInstance(lastModelForShot);
      double distance = instance.lastDistanceToShoot;
      DistanceToArmAngleModel.getInstance(lastModelForShot).updateModel(
        new double[] {distance, instance.applyFunction(distance) + Constants.Misc.OPERATOR_ANGLE_CORRECTION},
        true);
    }));

    aimedToLow.onTrue(new InstantCommand(() -> {
      DistanceToArmAngleModel instance = DistanceToArmAngleModel.getInstance(lastModelForShot);
      double distance = instance.lastDistanceToShoot;
      DistanceToArmAngleModel.getInstance(lastModelForShot).updateModel(
        new double[] {distance, instance.applyFunction(distance) - Constants.Misc.OPERATOR_ANGLE_CORRECTION},
        true);
    }));

    aimedJustRight.onTrue(new InstantCommand(() -> {
      DistanceToArmAngleModel instance = DistanceToArmAngleModel.getInstance(lastModelForShot);
      double distance = instance.lastDistanceToShoot;
      DistanceToArmAngleModel.getInstance(lastModelForShot).updateModel(
        new double[] {distance, instance.applyFunction(distance)},
        false);
    }));

    
    prep90.onTrue(new InstantCommand(()->RobotContainer.arm.setArmPivot(Rotation2d.fromDegrees(0))));   
    
    //TODO: Does this work?
    cleanUpMode.toggleOnTrue(new ToggleCleanUpCommand());
  }

  private void addShuffleBoardData() {
    SmartDashboard.putData(new ManualShootCommand(loader, arm));
    SmartDashboard.putData(new ClimberTuneCommand(climber));
    SmartDashboard.putData("Zero Left", new InstantCommand(()->climber.setLeftRotor(Rotation2d.fromDegrees(0))));
    SmartDashboard.putData("Zero Right", new InstantCommand(()->climber.setRightRotor(Rotation2d.fromDegrees(0))));
      // SmartDashboard.putData(new ClimberTestCommand(climber));
    // SmartDashboard.putData("Amp Prep", new ToNewAmpAdj());
    // SmartDashboard.putData("Amp Score", new ToNewAmp());
    // SmartDashboard.putData(new NoteToAmpOut());

    SmartDashboard.putData(new TestServoCommand(climber));
  }

  private void configureEvents() {   
    NamedCommands.registerCommand("ShootPrep", new InstantCommand(() -> Governor.setRobotState(RobotState.SHOOT_PREP, true)));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> Governor.setRobotState(RobotState.INTAKE, true)));
    NamedCommands.registerCommand("ShootWait", new SequentialCommandGroup(
      new WaitUntilCommand(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
          return sensors.getShooterSensor() && Governor.getRobotState() == RobotState.SHOOT_PREP;
      }
    }).withTimeout(3),
      new AimToSpeakerCommand(drive, joysticks),
      new InstantCommand(() -> Governor.setRobotState(RobotState.SHOOT, true)),
      new WaitCommand(0.2),
      new WaitUntilCommand(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return Governor.getDesiredRobotState() != RobotState.SHOOT;
        }
      }).withTimeout(3),
      new InstantCommand(() -> Governor.setRobotState(RobotState.INTAKE, true))
    ));
    NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
      new AimToSpeakerCommand(drive, joysticks),
      new InstantCommand(() -> Governor.setRobotState(RobotState.SHOOT, true)),
      new WaitCommand(0.2),
      new WaitUntilCommand(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return Governor.getDesiredRobotState() != RobotState.SHOOT;
        }
      }).withTimeout(3), //Consider adding additional loader sensor
      new InstantCommand(() -> Governor.setRobotState(RobotState.INTAKE, true))
    ));

    NamedCommands.registerCommand("P3Check", new P3Check());
    NamedCommands.registerCommand("P4Check", new P4Check());
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

  public static void writeRegressionFile(String fileName) {
    try {
      ArrayList<double[]> points = DistanceToArmAngleModel.getInstance(fileName).getUntransformedPoints();

            FileWriter fileWriter = new FileWriter(new File("U/regressionModel/" + fileName.split("\\.")[0] + Timer.getFPGATimestamp() + ".txt"));

            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

            bufferedWriter.flush();

            bufferedWriter.write(DistanceToArmAngleModel.getInstance(fileName).getEquation() + "\n");

            for(int i = 0; i < points.size(); i++) {
                bufferedWriter.write(points.get(i)[0] + " " + points.get(i)[1]);
                if(i != points.size() - 1) bufferedWriter.write("\n");
            }

            bufferedWriter.close();
            System.out.println("yay");
        } catch(Exception e) {
            System.out.println("UH OH");
            System.out.println(e);
        }
  }

}
