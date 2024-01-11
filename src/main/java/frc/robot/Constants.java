package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
  
    public static final double TAU = 2 * Math.PI;

    public static class Drive {
      public static final String CANBUS = "";

        // Ratio of the outer turn mechanism to the rotor
        public static final double TURN_GEAR_RATIO = 1;
        public static final double MOVE_GEAR_RATIO = 1;

      public static final double WIDTH = Units.inchesToMeters(1);
      public static final double LENGTH = Units.inchesToMeters(1);
      public static final double DIAGONAL = Math.sqrt(WIDTH*WIDTH + LENGTH*LENGTH)/2;

      public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WIDTH/2, -LENGTH/2),
        new Translation2d(WIDTH/2, LENGTH/2),
        new Translation2d(-WIDTH/2, LENGTH/2),
        new Translation2d(-WIDTH/2, -LENGTH/2)
      );

      public static final double WHEELRADIUS = Units.inchesToMeters(1);
      
      public static final double MAX_VELOCITY = 0; // MPS
      public static final double MAX_ROTATION_VELOCITY = 0; // RadPS

      public static final SwerveModuleConstants MOD0_CONSTANTS = new SwerveModuleConstants()
      .withCANcoderId(0)
      .withCANcoderOffset(0)
      .withCouplingGearRatio(0)
      .withDriveFrictionVoltage(0)
      .withDriveInertia(0)
      .withDriveMotorClosedLoopOutput(null)
      .withDriveMotorGains(new Slot0Configs())
      .withDriveMotorGearRatio(0)
      .withDriveMotorId(0)
      .withDriveMotorInverted(false)
      .withFeedbackSource(SteerFeedbackType.FusedCANcoder)  //<-- We're probably doing this
      .withLocationX(0)
      .withLocationY(0)
      .withSlipCurrent(0)
      .withSpeedAt12VoltsMps(0)
      .withSteerFrictionVoltage(0)
      .withSteerInertia(0)
      .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage) //IDK Test later
      .withSteerMotorGains(new Slot0Configs())
      .withSteerMotorGearRatio(0)
      .withSteerMotorId(0)
      .withSteerMotorInverted(false)
      .withWheelRadius(0);

      public static final SwerveModuleConstants MOD1_CONSTANTS = new SwerveModuleConstants()
      .withCANcoderId(0)
      .withCANcoderOffset(0)
      .withCouplingGearRatio(0)
      .withDriveFrictionVoltage(0)
      .withDriveInertia(0)
      .withDriveMotorClosedLoopOutput(null)
      .withDriveMotorGains(new Slot0Configs())
      .withDriveMotorGearRatio(0)
      .withDriveMotorId(0)
      .withDriveMotorInverted(false)
      .withFeedbackSource(SteerFeedbackType.FusedCANcoder)  //<-- We're probably doing this
      .withLocationX(0)
      .withLocationY(0)
      .withSlipCurrent(0)
      .withSpeedAt12VoltsMps(0)
      .withSteerFrictionVoltage(0)
      .withSteerInertia(0)
      .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage) //IDK Test later
      .withSteerMotorGains(new Slot0Configs())
      .withSteerMotorGearRatio(0)
      .withSteerMotorId(0)
      .withSteerMotorInverted(false)
      .withWheelRadius(0);
      
      public static final SwerveModuleConstants MOD2_CONSTANTS = new SwerveModuleConstants()
      .withCANcoderId(0)
      .withCANcoderOffset(0)
      .withCouplingGearRatio(0)
      .withDriveFrictionVoltage(0)
      .withDriveInertia(0)
      .withDriveMotorClosedLoopOutput(null)
      .withDriveMotorGains(new Slot0Configs())
      .withDriveMotorGearRatio(0)
      .withDriveMotorId(0)
      .withDriveMotorInverted(false)
      .withFeedbackSource(SteerFeedbackType.FusedCANcoder)  //<-- We're probably doing this
      .withLocationX(0)
      .withLocationY(0)
      .withSlipCurrent(0)
      .withSpeedAt12VoltsMps(0)
      .withSteerFrictionVoltage(0)
      .withSteerInertia(0)
      .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage) //IDK Test later
      .withSteerMotorGains(new Slot0Configs())
      .withSteerMotorGearRatio(0)
      .withSteerMotorId(0)
      .withSteerMotorInverted(false)
      .withWheelRadius(0);
      
      public static final SwerveModuleConstants MOD3_CONSTANTS = new SwerveModuleConstants()
      .withCANcoderId(0)
      .withCANcoderOffset(0)
      .withCouplingGearRatio(0)
      .withDriveFrictionVoltage(0)
      .withDriveInertia(0)
      .withDriveMotorClosedLoopOutput(null)
      .withDriveMotorGains(new Slot0Configs())
      .withDriveMotorGearRatio(0)
      .withDriveMotorId(0)
      .withDriveMotorInverted(false)
      .withFeedbackSource(SteerFeedbackType.FusedCANcoder)  //<-- We're probably doing this
      .withLocationX(0)
      .withLocationY(0)
      .withSlipCurrent(0)
      .withSpeedAt12VoltsMps(0)
      .withSteerFrictionVoltage(0)
      .withSteerInertia(0)
      .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage) //IDK Test later
      .withSteerMotorGains(new Slot0Configs())
      .withSteerMotorGearRatio(0)
      .withSteerMotorId(0)
      .withSteerMotorInverted(false)
      .withWheelRadius(0);
    }

    public static final class Joystick {
        public static final int LEFT_JOYSTICK_PORT = 1;
        public static final int RIGHT_JOYSTICK_PORT = 0;
        public static final int OPERATOR_PORT = 2;
    
        public static final double MOVE_DEAD_ZONE = 0.18;
        public static final double TURN_DEAD_ZONE = 0.1;
    
        public static final double ANGLE_DEAD_ZONE = Constants.TAU / 36;
    
        public static final double MOVE_SENSITIVITY = 1.5;
        public static final double TURN_SENSITIVITY = 1;
    
        public static final double MANUAL_EXTEND_DEADZONE = 0.1;
        public static final double MANUAL_PIVOT_DEADZONE = 0.1;
    
        public static final double MANUAL_EXTEND_OUT_SENSITIVITY = 0.18;
        public static final double MANUAL_EXTEND_IN_SENSITIVITY = 0.09;
        public static final double MANUAL_PIVOT_UP_SENSITIVITY = 0.12;
        public static final double MANUAL_PIVOT_DOWN_SENSITIVITY = 0.07;
        public static final double MANUAL_WRIST_SENSITIVITY = 0.5;
      }

      public static final class Misc {

        public static final int GYRO_PORT = 0;

      }

}
