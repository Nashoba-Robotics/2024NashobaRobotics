package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.TimedRobot;
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
import frc.robot.subsystems.apriltags.AprilTagManager;

public class Robot extends LoggedRobot {

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "2024NashobaRobotics"); // Set a metadata value

    // if(isReal()) {
    //     Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    //     Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    //     new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    // } else {
    //     setUseTiming(false); // Run as fast as possible
    //     String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //     Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.


    
    m_robotContainer = new RobotContainer();
    Tabs.addTab("April Tags");  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  }

  @Override
  public void teleopPeriodic() {
    try {
            Optional<EstimatedRobotPose> pos = AprilTagManager.getInstance().getEstimatedGlobalPose();
            Tabs.putBoolean("April Tags", "Has target", AprilTagManager.getInstance().hasTargets());
            if(pos.isPresent()) {
                EstimatedRobotPose estimatedPos = pos.get();
                SmartDashboard.putNumber("X", estimatedPos.estimatedPose.getX());

            }

        } catch (IOException e) {
            Tabs.putBoolean("April Tags", "Threw", true);
        }
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
