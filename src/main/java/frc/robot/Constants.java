package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
  
    public static final double TAU = 2 * Math.PI;

    public static class Drive {
      public static final String CANBUS = "rio";

      public static final double WIDTH = Units.inchesToMeters(20.75);
      public static final double LENGTH = Units.inchesToMeters(20.75);
      public static final double DIAGONAL = Math.sqrt(WIDTH*WIDTH + LENGTH*LENGTH)/2;

      public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WIDTH/2, -LENGTH/2),
        new Translation2d(WIDTH/2, LENGTH/2),
        new Translation2d(-WIDTH/2, LENGTH/2),
        new Translation2d(-WIDTH/2, -LENGTH/2)
      );

      public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
      
      public static final double MAX_VELOCITY = 4.23; // MPS
      public static final double MAX_ROTATION_VELOCITY = 10; // RadPS

      public static final Slot0Configs steerGains = new Slot0Configs()
          .withKP(100).withKI(0).withKD(0)
          .withKS(0).withKV(2.7272).withKA(0);
      private static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(0.1).withKI(0).withKD(0.001)
          .withKS(0).withKV(0.1139).withKA(0);

      private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
      private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

      private static final double kSlipCurrentA = 3000000;

      public static final double kSpeedAt12VoltsMps = 4.23;

      private static final double kCoupleRatio = 3.5714285714285716;
//       private static final double kCoupleRatio = 0;

      public static final double kDriveGearRatio = 8.142857142857142;
      public static final double kSteerGearRatio = 21.428571428571427;
      private static final double kWheelRadiusInches = 2;


      private static final boolean kSteerMotorReversed = true;
      private static final boolean kInvertLeftSide = false;
      private static final boolean kInvertRightSide = true;

      private static final String kCANbusName = "rio";
      private static final int kPigeonId = 0;

      // These are only used for simulation
      private static final double kSteerInertia = 0.00001;
      private static final double kDriveInertia = 0.001;
      // Simulated voltage necessary to overcome friction
      private static final double kSteerFrictionVoltage = 0.25;
      private static final double kDriveFrictionVoltage = 0.25;

      private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withWheelRadius(kWheelRadiusInches)
              .withSlipCurrent(kSlipCurrentA)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(kCoupleRatio)
              .withSteerMotorInverted(kSteerMotorReversed);

      // Front Left
      private static final int kFrontLeftDriveMotorId = 1;
      private static final int kFrontLeftSteerMotorId = 5;
      private static final int kFrontLeftEncoderId = 1;
      private static final double kFrontLeftEncoderOffset = 0.461914;

      private static final double kFrontLeftXPosInches = 10.375;
      private static final double kFrontLeftYPosInches = 10.375;

      // Front Right
      private static final int kFrontRightDriveMotorId = 0;
      private static final int kFrontRightSteerMotorId = 4;
      private static final int kFrontRightEncoderId = 0;
      private static final double kFrontRightEncoderOffset = -0.108887;

      private static final double kFrontRightXPosInches = 10.375;
      private static final double kFrontRightYPosInches = -10.375;

      // Back Left
      private static final int kBackLeftDriveMotorId = 2;
      private static final int kBackLeftSteerMotorId = 6;
      private static final int kBackLeftEncoderId = 2;
      private static final double kBackLeftEncoderOffset = -0.443848;

      private static final double kBackLeftXPosInches = -10.375;
      private static final double kBackLeftYPosInches = 10.375;

      // Back Right
      private static final int kBackRightDriveMotorId = 3;
      private static final int kBackRightSteerMotorId = 7;
      private static final int kBackRightEncoderId = 3;
      private static final double kBackRightEncoderOffset = 0.209229;

      private static final double kBackRightXPosInches = -10.375;
      private static final double kBackRightYPosInches = -10.375;

      public static final SwerveModuleConstants MOD1_CONSTANTS = ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
      public static final SwerveModuleConstants MOD0_CONSTANTS = ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
      public static final SwerveModuleConstants MOD2_CONSTANTS = ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
      public static final SwerveModuleConstants MOD3_CONSTANTS = ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

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
