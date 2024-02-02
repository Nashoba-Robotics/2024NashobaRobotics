package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.drive.DriveSubsystem;

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

    new AprilTagManager();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if(AprilTagManager.hasTarget() 
      && AprilTagManager.getAmbiguity() <= 0.2 
      && AprilTagManager.getRobotPos() != null
      )
        RobotContainer.drive.updateOdometryWithVision(AprilTagManager.getRobotPos().toPose2d(), AprilTagManager.getTimestamp());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    robotContainer.getAutoCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    CommandScheduler.getInstance().setDefaultCommand(
      RobotContainer.drive,
      new DriveCommand(RobotContainer.drive, RobotContainer.joysticks)
      );
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
