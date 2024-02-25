package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Governor.RobotState;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.President;
import frc.robot.subsystems.apriltags.AprilTagManager;

public class Robot extends LoggedRobot {

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "2024NashobaRobotics"); // Set a metadata value

    if(isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    robotContainer = new RobotContainer();
    Tabs.addTab("April Tags");  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if(AprilTagManager.hasLeftTarget()
        && AprilTagManager.getLeftAmbiguity() <= 0.2
        && AprilTagManager.getLeftRobotPos() != null)
          RobotContainer.drive.updateOdometryWithVision(AprilTagManager.getLeftRobotPos().toPose2d(), AprilTagManager.getLeftTimestamp());
    if(AprilTagManager.hasRightTarget()
        && AprilTagManager.getRightAmbiguity() <= 0.2
        && AprilTagManager.getRightRobotPos() != null)
          RobotContainer.drive.updateOdometryWithVision(AprilTagManager.getRightRobotPos().toPose2d(), AprilTagManager.getRightTimestamp());

    double y = Constants.Field.getSpeakerPos().getZ()-Constants.Robot.SHOOTER_HEIGHT;
    double dist = RobotContainer.drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
    double angle = -Math.atan2(y, dist);
    Logger.recordOutput("Arm Aim Angle", angle*360/Constants.TAU);
    Logger.recordOutput("Aim Distance", dist);

    SmartDashboard.putString("RobotState", Governor.getRobotState().toString());
    SmartDashboard.putString("QueuedState", Governor.getQueuedState().toString());
    Logger.recordOutput("RobotState/RobotState", Governor.getRobotState().toString());
    Logger.recordOutput("RobotState/QueuedState", Governor.getQueuedState().toString());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(new InstantCommand(
      ()->{
        RobotContainer.arm.setArmPivotRotor(Presets.Arm.INTAKE_POS);
        RobotContainer.loader.setPivotRotor(Presets.Loader.INTAKE_POS);
      }, RobotContainer.arm, RobotContainer.loader
    ));
    robotContainer.getAutoCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //Cancels everything from auto
    CommandScheduler.getInstance().cancelAll();

    CommandScheduler.getInstance().setDefaultCommand(
      RobotContainer.drive,
      new DriveCommand(RobotContainer.drive, RobotContainer.joysticks)
      );

    CommandScheduler.getInstance().schedule(new President());
    Governor.setRobotState(RobotState.NEUTRAL, true);
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
