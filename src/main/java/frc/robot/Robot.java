package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Scanner;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Governor.RobotState;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.President;
import frc.robot.commands.auto.Dictator;
import frc.robot.lib.util.DistanceToArmAngleModel;
import frc.robot.subsystems.apriltags.AprilTagManager;

public class Robot extends LoggedRobot {

  public RobotContainer robotContainer;
  private Timer jank = new Timer();

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "2024NashobaRobotics"); // Set a metadata value

    if(isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev).setSwitchableChannel(true); // Enables power distribution logging
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    robotContainer = new RobotContainer();
    Tabs.addTab("April Tags");

    jank.start();    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Pose2d leftPose2d = AprilTagManager.getLeftRobotPos().toPose2d();
    Pose2d rightPose2d = AprilTagManager.getRightRobotPos().toPose2d();

    double leftError = leftPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();
    double rightError = rightPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();

    Logger.recordOutput("LeftErrorDist", leftError);
    Logger.recordOutput("rightErrorDist", rightError);

    // if(jank.get() >= 0.005){
      // if(DriverStation.isAutonomous()) {
      //   if(AprilTagManager.hasLeftTarget()
      //       && AprilTagManager.getLeftAmbiguity() <= 0.15
      //       && AprilTagManager.getLeftRobotPos() != null
      //       && leftError < 1
      //       && leftPose2d.getX() > 0 && leftPose2d.getX() < Constants.Field.LENGTH
      //       && leftPose2d.getY() > 0 && leftPose2d.getY() < Constants.Field.WIDTH)
      //         RobotContainer.drive.updateOdometryWithVision(leftPose2d, AprilTagManager.getLeftTimestamp());
      //   if(AprilTagManager.hasRightTarget()
      //       && AprilTagManager.getRightAmbiguity() <= 0.15
      //       && AprilTagManager.getRightRobotPos() != null
      //       && rightError < 1
      //       && rightPose2d.getX() > 0 && rightPose2d.getX() < Constants.Field.LENGTH
      //       && rightPose2d.getY() > 0 && rightPose2d.getY() < Constants.Field.WIDTH)
      //         RobotContainer.drive.updateOdometryWithVision(rightPose2d, AprilTagManager.getRightTimestamp());
      // } else {
      //   if(AprilTagManager.hasLeftTarget()
      //       && AprilTagManager.getLeftAmbiguity() <= 0.15
      //       && AprilTagManager.getLeftRobotPos() != null
      //       && leftError < 5
      //       && leftPose2d.getX() > 0 && leftPose2d.getX() < Constants.Field.LENGTH
      //       && leftPose2d.getY() > 0 && leftPose2d.getY() < Constants.Field.WIDTH)
      //         RobotContainer.drive.updateOdometryWithVision(leftPose2d, AprilTagManager.getLeftTimestamp());
      //   if(AprilTagManager.hasRightTarget()
      //       && AprilTagManager.getRightAmbiguity() <= 0.15
      //       && AprilTagManager.getRightRobotPos() != null
      //       && rightError < 5
      //       && rightPose2d.getX() > 0 && rightPose2d.getX() < Constants.Field.LENGTH
      //       && rightPose2d.getY() > 0 && rightPose2d.getY() < Constants.Field.WIDTH)
      //         RobotContainer.drive.updateOdometryWithVision(rightPose2d, AprilTagManager.getRightTimestamp());
      // }
    //   jank.restart();
    // }

    double dist = RobotContainer.drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
    Logger.recordOutput("Aim Distance", dist);

    SmartDashboard.putString("RobotState", Governor.getRobotState().toString());
    SmartDashboard.putString("QueuedState", Governor.getQueuedState().toString());
    Logger.recordOutput("RobotState/RobotState", Governor.getRobotState().toString());
    Logger.recordOutput("RobotState/QueuedState", Governor.getQueuedState().toString());
  }

  @Override
  public void disabledInit() {
    try {
      ArrayList<double[]> points = DistanceToArmAngleModel.getInstance().getUntransformedPoints();

            FileWriter fileWriter = new FileWriter(new File("U/distanceToArmAngle.txt"));

            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

            bufferedWriter.write(DistanceToArmAngleModel.getInstance().getEquation() + "\n");

            for(int i = 0; i < points.size(); i++) {
                bufferedWriter.write(points.get(i)[0] + " " + points.get(i)[1]);
                if(i != points.size() - 1) bufferedWriter.write("\n");
            }

            bufferedWriter.close();
            System.out.println("yay");
        } catch(Exception e) {
            System.out.println("UH OH");
        }
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
    
    
    CommandScheduler.getInstance().schedule(new Dictator());
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

    // CommandScheduler.getInstance().schedule(new President());
    // Governor.setRobotState(RobotState.NEUTRAL, true);


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
