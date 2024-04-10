package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Governor;
import frc.robot.Governor.RobotState;
import frc.robot.lib.math.NRUnits;
import frc.robot.lib.util.JoystickValues;
import frc.robot.lib.util.MoveMath;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveState;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class AimToSpeakerCommand extends Command{
    DriveSubsystem drive;
    JoystickSubsystem joysticks;

    PIDController pidController;
    TrapezoidProfile feedForwardProfile;

    State startStateUnconstrained;
    Rotation2d startAngleConstrained;

    Rotation2d targetAngle;

    Timer t;

    JoystickValues leftJoystickValues;
    JoystickValues rightJoystickValues;

    boolean flag;


    public AimToSpeakerCommand(DriveSubsystem drive, JoystickSubsystem joysticks){
        targetAngle = new Rotation2d();
        this.drive = drive;
        this.joysticks = joysticks;

        feedForwardProfile = new TrapezoidProfile(new Constraints(5, 20));
        t = new Timer();

        pidController = new PIDController(0.5, 0, 0);
        pidController.enableContinuousInput(-Constants.TAU/2, Constants.TAU/2);

        addRequirements(drive);

        leftJoystickValues = new JoystickValues(0, 0);
        rightJoystickValues = new JoystickValues(0, 0);

        flag = false;

    }

    @Override
    public void initialize() {
        Translation3d noteVector = MoveMath.getBallisticTrajectory();

        Logger.recordOutput("Note Pos Vector", noteVector);
        Translation3d targetPos = new Translation3d(
            Constants.Field.getSpeakerPos().getX()-noteVector.getX(), 
            Constants.Field.getSpeakerPos().getY()-noteVector.getY(), 
            Constants.Field.getSpeakerPos().getZ()-noteVector.getZ());


        targetAngle = Rotation2d.fromRadians(Math.atan2(
            targetPos.getY() - drive.getPose().getY(),
            targetPos.getX() - drive.getPose().getX()));


        startStateUnconstrained = new State(drive.getYaw().getRadians(), drive.getZVelocity());
        startAngleConstrained = drive.getPose().getRotation();
        t.restart();
        pidController.setP(0.5);
        flag = false;
    }

    @Override
    public void execute() {

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        leftJoystickValues = joysticks.getLeftJoystickValues()
            .shape(Constants.Joystick.MOVE_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY)
            .swap()
            .applyAngleDeadzone(Constants.Joystick.ANGLE_DEAD_ZONE);
        rightJoystickValues = joysticks.getRightJoystickValues()
            .shape(Constants.Joystick.TURN_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY);

        // leftJoystickValues = joysticks.getLeftOperatorValues()
        //     .shape(Constants.Joystick.MOVE_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY)
        //     .swap()
        //     .applyAngleDeadzone(Constants.Joystick.ANGLE_DEAD_ZONE);
        // rightJoystickValues = joysticks.getRightOperatorValues()
        //     .shape(Constants.Joystick.TURN_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY);

        chassisSpeeds.vxMetersPerSecond = leftJoystickValues.x * Constants.Drive.MAX_VELOCITY;
        chassisSpeeds.vyMetersPerSecond = leftJoystickValues.y * Constants.Drive.MAX_VELOCITY;


        // if(feedForwardProfile.isFinished(t.get())) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                targetAngle = Rotation2d.fromRadians(MoveMath.getShootWhileMoveBallistics2()[0] + Math.PI);
        } else {
            targetAngle = Rotation2d.fromRadians(MoveMath.getShootWhileMoveBallistics2()[0]);
        }
        Translation3d noteVector = MoveMath.getBallisticTrajectory();
        Translation3d targetPos = new Translation3d(
            Constants.Field.getSpeakerPos().getX()-noteVector.getX(), 
            Constants.Field.getSpeakerPos().getY()-noteVector.getY(), 
            Constants.Field.getSpeakerPos().getZ()-noteVector.getZ());


            // targetAngle = Rotation2d.fromRadians(Math.atan2(
            // targetPos.getY() - drive.getPose().getY(),
            // targetPos.getX() - drive.getPose().getX()));
            if(!flag) {
                pidController.setP(6);
                flag = true;
            }
        // }

        double diff = targetAngle.getRadians() - startAngleConstrained.getRadians();

        State goalState = Math.abs(diff) <= Constants.TAU/2 ?
            new State(startStateUnconstrained.position + diff, 0) :
            new State(startStateUnconstrained.position - (Constants.TAU - Math.abs(diff))*Math.signum(diff), 0);

        State setState = feedForwardProfile.calculate(t.get(), startStateUnconstrained, goalState);

        double rotSpeed = feedForwardProfile.isFinished(t.get()) || true ? pidController.calculate(drive.getPose().getRotation().getRadians(), targetAngle.getRadians()) :
        setState.velocity
        + pidController.calculate(drive.getYaw().getRadians(), setState.position);

        Logger.recordOutput("Current Angle", setState.position);
        chassisSpeeds.omegaRadiansPerSecond = rotSpeed;

        drive.set(chassisSpeeds);

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if(DriverStation.isAutonomous()) {
            return feedForwardProfile.isFinished(t.get());
        } else
        return 
        // Governor.getRobotState() != RobotState.SHOOT && Governor.getRobotState() != RobotState.SHOOT_PREP && Governor.getRobotState() != RobotState.TRANSITION
        // || 
        Math.abs(joysticks.getRightJoystickValues().x) >= 0.03;
    }
}
