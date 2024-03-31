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
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    SmartDashboard.putNumber("High Shuttle Speed", Presets.Arm.HIGH_SHUTTLE_SPEED.getRadians());
    SmartDashboard.putNumber("Low Shuttle Speed", Presets.Arm.LOW_SHUTTLE_SPEED.getRadians());
    RobotContainer.odometryFlag = false;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Pose2d leftPose2d = AprilTagManager.getLeftRobotPos().toPose2d();
    Pose2d rightPose2d = AprilTagManager.getRightRobotPos().toPose2d();

    double leftError = leftPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();
    double rightError = rightPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();

    Pose2d backLeftPose2d = AprilTagManager.getBackLeftPos().toPose2d();
    Pose2d backRightPose2d = AprilTagManager.getBackRightPos().toPose2d();

    double backLeftError = backLeftPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();
    double backRightError = backRightPose2d.relativeTo(RobotContainer.drive.getPose()).getTranslation().getNorm();

    Logger.recordOutput("LeftErrorDist", leftError);
    Logger.recordOutput("rightErrorDist", rightError);

      if(DriverStation.isAutonomous()) {
        if(AprilTagManager.hasLeftTarget()
            && AprilTagManager.getLeftAmbiguity() <= 0.15
            && AprilTagManager.getLeftRobotPos() != null
            && leftError < 1
            && leftPose2d.getX() > 0 && leftPose2d.getX() < Constants.Field.LENGTH
            && leftPose2d.getY() > 0 && leftPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(leftPose2d, AprilTagManager.getLeftTimestamp());

        if(AprilTagManager.hasRightTarget()
            && AprilTagManager.getRightAmbiguity() <= 0.15
            && AprilTagManager.getRightRobotPos() != null
            && rightError < 1
            && rightPose2d.getX() > 0 && rightPose2d.getX() < Constants.Field.LENGTH
            && rightPose2d.getY() > 0 && rightPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(rightPose2d, AprilTagManager.getRightTimestamp());

              if(AprilTagManager.hasBackLeftTarget()
            && AprilTagManager.getBackLeftAmbiguity() <= 0.15
            && AprilTagManager.getBackLeftPos() != null
            && backLeftError < 1
            && backLeftPose2d.getX() > 0 && backLeftPose2d.getX() < Constants.Field.LENGTH
            && backLeftPose2d.getY() > 0 && backLeftPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(backLeftPose2d, AprilTagManager.getBackLeftTimestamp());

              if(AprilTagManager.hasBackRightTarget()
            && AprilTagManager.getBackRightAmbiguity() <= 0.15
            && AprilTagManager.getBackRightPos() != null
            && backRightError < 1
            && backRightPose2d.getX() > 0 && backRightPose2d.getX() < Constants.Field.LENGTH
            && backRightPose2d.getY() > 0 && backRightPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(backRightPose2d, AprilTagManager.getBackRightTimestamp());
      } else if (!RobotContainer.drive.overrideVisionOdo){
        if(AprilTagManager.hasLeftTarget()
            && AprilTagManager.getLeftAmbiguity() <= 0.15
            && AprilTagManager.getLeftRobotPos() != null
            && leftError < 5
            && leftPose2d.getX() > 0 && leftPose2d.getX() < Constants.Field.LENGTH
            && leftPose2d.getY() > 0 && leftPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(leftPose2d, AprilTagManager.getLeftTimestamp());
        if(AprilTagManager.hasRightTarget()
            && AprilTagManager.getRightAmbiguity() <= 0.15
            && AprilTagManager.getRightRobotPos() != null
            && rightError < 5
            && rightPose2d.getX() > 0 && rightPose2d.getX() < Constants.Field.LENGTH
            && rightPose2d.getY() > 0 && rightPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(rightPose2d, AprilTagManager.getRightTimestamp());

        if(AprilTagManager.hasBackLeftTarget()
            && AprilTagManager.getBackLeftAmbiguity() <= 0.15
            && AprilTagManager.getBackLeftPos() != null
            && backLeftError < 2
            && backLeftPose2d.getX() > 0 && backLeftPose2d.getX() < Constants.Field.LENGTH
            && backLeftPose2d.getY() > 0 && backLeftPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(backLeftPose2d, AprilTagManager.getBackLeftTimestamp());

              if(AprilTagManager.hasBackRightTarget()
            && AprilTagManager.getBackRightAmbiguity() <= 0.15
            && AprilTagManager.getBackRightPos() != null
            && backRightError < 2
            && backRightPose2d.getX() > 0 && backRightPose2d.getX() < Constants.Field.LENGTH
            && backRightPose2d.getY() > 0 && backRightPose2d.getY() < Constants.Field.WIDTH)
              RobotContainer.drive.updateOdometryWithVision(backRightPose2d, AprilTagManager.getBackRightTimestamp());
      }
    
      SmartDashboard.putBoolean("ODOFlag", RobotContainer.odometryFlag);

    double dist = RobotContainer.drive.getPose().getTranslation().getDistance(Constants.Field.getSpeakerPos().toTranslation2d());
    Logger.recordOutput("Regression/Aim Distance", dist);
    Logger.recordOutput("Regression/ArmSetAngle", DistanceToArmAngleModel.getInstance(RobotContainer.lastModelForShot).applyFunction(dist));

    SmartDashboard.putString("RobotState", Governor.getRobotState().toString());
    SmartDashboard.putString("QueuedState", Governor.getQueuedState().toString());
    Logger.recordOutput("RobotState/RobotState", Governor.getRobotState().toString());
    Logger.recordOutput("RobotState/QueuedState", Governor.getQueuedState().toString());
    Logger.recordOutput("RobotState/LastState", Governor.getDesiredRobotState().toString());
    Logger.recordOutput("RobotState/DesiredState", Governor.getDesiredRobotState());

    Logger.recordOutput("LastRegressionModel", RobotContainer.lastModelForShot);
  }

  @Override
  public void disabledInit() {
    for(String fileName : Constants.FileNames.ArmAngleFiles) {
      RobotContainer.writeRegressionFile(
        (DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue ? "blue/" : "red/") +
        fileName);
    }
    // RobotContainer.writeRegressionFile(Constants.FileNames.ARM_ANGLE_CLOSE);
    // RobotContainer.writeRegressionFile(Constants.FileNames.ARM_ANGLE_FAR_AMP);
    // RobotContainer.writeRegressionFile(Constants.FileNames.ARM_ANGLE_FAR_SOURCE);
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
    RobotContainer.odometryFlag = true;

    RobotContainer.drive.enableStatorLimits(false);
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

    RobotContainer.drive.overrideVisionOdo = false;

    RobotContainer.drive.enableStatorLimits(true);
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
