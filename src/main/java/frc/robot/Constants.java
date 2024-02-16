package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {
      public static final double TAU = 2 * Math.PI;
      public static final double PEAK_VOLTAGE = 12;

      public static final class AprilTags{
            public static final String CAMERA1_NAME = "Yi's_Little_Buddy";
            public static final String CAMERA2_NAME = "Ben's_Little_Buddy";

            /* For PhotonEstimator
            *             ^ 
            *             |
            *             Z        
            *      --------------
            *      |            |
            *      |            |
            *<-- X |     *Y     |
            *      |            |
            *      |            |
            *      --------------
            */
            public static final Transform3d ROBOT_TO_CAMERA1 = new Transform3d(0, 0, 0, new Rotation3d(0, -18./360*TAU, 0));
            public static final Transform3d ROBOT_TO_CAMERA2 = new Transform3d(0,0,0, new Rotation3d());

            //With the Layout paths, REMEMBER you need to also upload the json file to the Photonvision GUI
            //This layout for some reason only works for the single tag estimation (as of 02/11/24) 
            public static final String LAYOUT_PATH = Filesystem.getDeployDirectory().getPath() + "/AprilTagPositions.json";

            public static final double getXSD(double distance) {
                  return 0.0312*distance - 0.0494;
            }

            public static final double getYSD(double distance) {
                  return 0.0656*distance - 0.129;
            }
      }

    public static final class Arm{
            public static final String CANBUS = "rio";

            public static final int PIVOT_PORT = 9;

            public static final int SHOOTER_PORT = 10;
            public static final int SHOOTER_PORT_2 = 11;

            public static final int SHOOTER_SENSOR_PORT = 0;
            public static final int LOADER_SENSOR_PORT = 1;

            public static final double SHOOTER_GEAR_RATIO = 0;
            public static final double PIVOT_GEAR_RATIO = 48. * 32/15;

            public static final double PIVOT_STATOR_CURRENT_LIMIT = 45.0;
            public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 15.0;

            public static final Rotation2d PIVOT_FORWARD_SOFT_LIMIT = Rotation2d.fromDegrees(120);
            public static final Rotation2d PIVOT_REVERSE_SOFT_LIMIT = Rotation2d.fromDegrees(-50);

            public static final double PIVOT_MOTION_MAGIC_ACCELERATION = 0.5;
            public static final double PIVOT_MOTION_MAGIC_CRUISE_VELOCITY = 0.95;
            public static final double PIVOT_MOTION_MAGIC_JERK = 0;

            public static final InvertedValue PIVOT_INVERTED = InvertedValue.CounterClockwise_Positive;
            public static final NeutralModeValue PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;

            public static final Slot0Configs PIVOT_PID = new Slot0Configs()
            .withKV(1).withKS(0.022).withKG(0.03).withGravityType(GravityTypeValue.Arm_Cosine)
            .withKP(60).withKI(0).withKD(0.6);

            public static final double SHOOTER_STATOR_CURRENT_LIMIT = 0;
            public static final double SHOOTER_SUPPLY_CURRENT_LIMIT = 0;

            public static final InvertedValue SHOOTER_INVERTED = InvertedValue.CounterClockwise_Positive;
            public static final NeutralModeValue SHOOTER_NEUTRAL_MODE = NeutralModeValue.Brake;

            public static final Slot0Configs SHOOTER_PID = new Slot0Configs()
            .withKV(0).withKS(0)
            .withKP(0).withKI(0).withKD(0);
      }

      public static class Climber{
            public static final String CANBUS = "rio";
            
            public static final int LEFT_CLIMBER_PORT = 0;
            public static final int RIGHT_CLIMBER_PORT = 0;

            public static final double STATOR_LIMIT = 0;
            public static final double GEAR_RATIO = 0;

            public static final double FORWARD_SOFT_LIMIT = 0;
            public static final double REVERSE_SOFT_LIMIT = 0;

            public static final Slot0Configs leftPID = new Slot0Configs()
            .withKS(0).withKV(0).withKA(0)
            .withKP(0).withKI(0).withKD(0.0); 
            public static final Slot0Configs rightPID = new Slot0Configs()
            .withKS(0).withKV(0).withKA(0)
            .withKP(0).withKI(0).withKD(0.0); 
      }

      public static class Drive {
            public static final String CANBUS = "rio";

            public static final double WIDTH = Units.inchesToMeters(22.75); // ~0.57785m
            public static final double LENGTH = Units.inchesToMeters(22.75);
            public static final double DIAGONAL = Math.sqrt(WIDTH*WIDTH + LENGTH*LENGTH)/2;

            public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WIDTH/2, -LENGTH/2),
            new Translation2d(WIDTH/2, LENGTH/2),
            new Translation2d(-WIDTH/2, LENGTH/2),
            new Translation2d(-WIDTH/2, -LENGTH/2)
            );

            public static final double MAX_VELOCITY = 3.70; // MPS
            public static final double MAX_ACCELERATION = 0;

            public static final double MAX_ROTATION_VELOCITY = 9.30; // RadPS
            public static final double MAX_ROTATION_ACCELERATION = 0;

            public static final Slot0Configs steerGains0 = new Slot0Configs()
            .withKP(20).withKI(0).withKD(0.0)
                  // .withKP(0).withKI(0).withKD(0)
            .withKS(0.25).withKV(2.615).withKA(0);

            public static final Slot0Configs steerGains1 = new Slot0Configs()
            .withKP(20).withKI(0).withKD(0.0)
            // .withKP(0).withKI(0).withKD(0)
            .withKS(0.27).withKV(2.590).withKA(0);
            
            public static final Slot0Configs steerGains2 = new Slot0Configs()
            .withKP(20).withKI(0).withKD(0.0)
            // .withKP(0).withKI(0).withKD(0)
            .withKS(0.28).withKV(2.600).withKA(0);

            public static final Slot0Configs steerGains3 = new Slot0Configs()
            .withKP(20).withKI(0).withKD(0.0)
            //     .withKP(0).withKI(0).withKD(0)
            .withKS(0.34).withKV(2.681).withKA(0); 



            //TODO: Change back
            private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.22).withKI(0).withKD(0)
            // .withKP(0).withKI(0).withKD(0)
            .withKS(0).withKV(0.1165).withKA(0);

            private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
            private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

            private static final double kSlipCurrentA = 60;

            public static final double kSpeedAt12VoltsMps = MAX_VELOCITY;

            // private static final double kCoupleRatio = 3.5714285714285716;
            private static final double kCoupleRatio = 0;

            public static final double kDriveGearRatio = 8.142857142857142;
            public static final double kSteerGearRatio = 21.428571428571427;
            // private static final double kWheelRadiusInches = 1.840; //Direction of resistence
            private static final double kWheelRadiusInches = 1.967; //Direction of less-resistence
            // private static final double kWheelRadiusInches = 1.925; //Comp

            public static final double WHEEL_RADIUS = Units.inchesToMeters(kWheelRadiusInches);

            private static final boolean kSteerMotorReversed = true;
            private static final boolean kInvertLeftSide = false;
            private static final boolean kInvertRightSide = true;

            // These are only used for simulation
            private static final double kSteerInertia = 0.00001;
            private static final double kDriveInertia = 0.001;
            // Simulated voltage necessary to overcome friction
            private static final double kSteerFrictionVoltage = 0.25;
            private static final double kDriveFrictionVoltage = 0.25;

            private static final SwerveModuleConstantsFactory ConstantCreator0 = new SwerveModuleConstantsFactory()
                  .withDriveMotorGearRatio(kDriveGearRatio)
                  .withSteerMotorGearRatio(kSteerGearRatio)
                  .withWheelRadius(kWheelRadiusInches)
                  .withSlipCurrent(kSlipCurrentA)
                  .withSteerMotorGains(steerGains0)
                  .withDriveMotorGains(driveGains)
                  .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                  .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                  .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                  .withSteerInertia(kSteerInertia)
                  .withDriveInertia(kDriveInertia)
                  .withSteerFrictionVoltage(kSteerFrictionVoltage)
                  .withDriveFrictionVoltage(kDriveFrictionVoltage)
                  .withFeedbackSource(SteerFeedbackType.SyncCANcoder)
                  .withCouplingGearRatio(kCoupleRatio)
                  .withSteerMotorInverted(kSteerMotorReversed);

            private static final SwerveModuleConstantsFactory ConstantCreator1 = new SwerveModuleConstantsFactory()
                  .withDriveMotorGearRatio(kDriveGearRatio)
                  .withSteerMotorGearRatio(kSteerGearRatio)
                  .withWheelRadius(kWheelRadiusInches)
                  .withSlipCurrent(kSlipCurrentA)
                  .withSteerMotorGains(steerGains1)
                  .withDriveMotorGains(driveGains)
                  .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                  .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                  .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                  .withSteerInertia(kSteerInertia)
                  .withDriveInertia(kDriveInertia)
                  .withSteerFrictionVoltage(kSteerFrictionVoltage)
                  .withDriveFrictionVoltage(kDriveFrictionVoltage)
                  .withFeedbackSource(SteerFeedbackType.SyncCANcoder)
                  .withCouplingGearRatio(kCoupleRatio)
                  .withSteerMotorInverted(kSteerMotorReversed);
            
            private static final SwerveModuleConstantsFactory ConstantCreator2 = new SwerveModuleConstantsFactory()
                  .withDriveMotorGearRatio(kDriveGearRatio)
                  .withSteerMotorGearRatio(kSteerGearRatio)
                  .withWheelRadius(kWheelRadiusInches)
                  .withSlipCurrent(kSlipCurrentA)
                  .withSteerMotorGains(steerGains2)
                  .withDriveMotorGains(driveGains)
                  .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                  .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                  .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                  .withSteerInertia(kSteerInertia)
                  .withDriveInertia(kDriveInertia)
                  .withSteerFrictionVoltage(kSteerFrictionVoltage)
                  .withDriveFrictionVoltage(kDriveFrictionVoltage)
                  .withFeedbackSource(SteerFeedbackType.SyncCANcoder)
                  .withCouplingGearRatio(kCoupleRatio)
                  .withSteerMotorInverted(kSteerMotorReversed);

            private static final SwerveModuleConstantsFactory ConstantCreator3 = new SwerveModuleConstantsFactory()
                  .withDriveMotorGearRatio(kDriveGearRatio)
                  .withSteerMotorGearRatio(kSteerGearRatio)
                  .withWheelRadius(kWheelRadiusInches)
                  .withSlipCurrent(kSlipCurrentA)
                  .withSteerMotorGains(steerGains3)
                  .withDriveMotorGains(driveGains)
                  .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                  .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                  .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                  .withSteerInertia(kSteerInertia)
                  .withDriveInertia(kDriveInertia)
                  .withSteerFrictionVoltage(kSteerFrictionVoltage)
                  .withDriveFrictionVoltage(kDriveFrictionVoltage)
                  .withFeedbackSource(SteerFeedbackType.SyncCANcoder)
                  .withCouplingGearRatio(kCoupleRatio)
                  .withSteerMotorInverted(kSteerMotorReversed);

            // Front Left
            private static final int kFrontLeftDriveMotorId = 5;
            private static final int kFrontLeftSteerMotorId = 1;
            private static final int kFrontLeftEncoderId = 1;
            private static final double kFrontLeftEncoderOffset = 0.461914;

            private static final double kFrontLeftXPosInches = 10.375;
            private static final double kFrontLeftYPosInches = 10.375;

            // Front Right
            private static final int kFrontRightDriveMotorId = 4;
            private static final int kFrontRightSteerMotorId = 0;
            private static final int kFrontRightEncoderId = 0;
            private static final double kFrontRightEncoderOffset = -0.108887;

            private static final double kFrontRightXPosInches = 10.375;
            private static final double kFrontRightYPosInches = -10.375;

            // Back Left
            private static final int kBackLeftDriveMotorId = 6;
            private static final int kBackLeftSteerMotorId = 2;
            private static final int kBackLeftEncoderId = 2;
            private static final double kBackLeftEncoderOffset = -0.443848;

            private static final double kBackLeftXPosInches = -10.375;
            private static final double kBackLeftYPosInches = 10.375;

            // Back Right
            private static final int kBackRightDriveMotorId = 7;
            private static final int kBackRightSteerMotorId = 3;
            private static final int kBackRightEncoderId = 3;
            private static final double kBackRightEncoderOffset = 0.209229;

            private static final double kBackRightXPosInches = -10.375;
            private static final double kBackRightYPosInches = -10.375;

            public static final SwerveModuleConstants MOD1_CONSTANTS = ConstantCreator1.createModuleConstants(
                  kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
            public static final SwerveModuleConstants MOD0_CONSTANTS = ConstantCreator0.createModuleConstants(
                  kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
            public static final SwerveModuleConstants MOD2_CONSTANTS = ConstantCreator2.createModuleConstants(
                  kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
            public static final SwerveModuleConstants MOD3_CONSTANTS = ConstantCreator3.createModuleConstants(
                  kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

      }

      public static final class Field {
            public static final Translation2d SPEAKER_POSITION = new Translation2d(0, 0);
            public static final Translation2d AMP_POSITION = new Translation2d(0, 0);
      }

      public static final class Joystick {
            public static final int DRIVER_PORT = 0;
            public static final int OPERATOR_PORT = 2;
    
            public static final double MOVE_DEAD_ZONE = 0.05;
            public static final double TURN_DEAD_ZONE = 0.1;
    
            public static final double ANGLE_DEAD_ZONE = Constants.TAU / 72;
    
            public static final double MOVE_SENSITIVITY = 1.5;
            public static final double TURN_SENSITIVITY = 1;
      }

      public static final class Loader {
            public static final String CANBUS = "rio";

            public static final int PIVOT_PORT = 12;
            public static final int ROLLER_PORT = 13;

            public static final int LOADER_SENSOR_PORT = 1;
            public static final int SHOOTER_SENSOR_PORT = 0;

            public static final double PIVOT_GEAR_RATIO = 9.*7*30/18;
            public static final double ROLLER_GEAR_RATIO = 7;

            public static final double PIVOT_STATOR_CURRENT_LIMIT = 20;
            public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 20;

            public static final Rotation2d PIVOT_FORWARD_SOFT_LIMIT = Rotation2d.fromDegrees(100);
            public static final Rotation2d PIVOT_REVERSE_SOFT_LIMIT = Rotation2d.fromDegrees(-30);

            public static final double PIVOT_MOTION_MAGIC_ACCELERATION = 2;   //2
            public static final double PIVOT_MOTION_MAGIC_CRUISE_VELOCITY = 0.95;
            public static final double PIVOT_MOTION_MAGIC_JERK = 0;

            public static final InvertedValue PIVOT_INVERTED = InvertedValue.CounterClockwise_Positive;
            public static final NeutralModeValue PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;

            public static final Slot0Configs PIVOT_PID = new Slot0Configs()
            .withKV(0.845).withKS(0.02)
            .withKP(45).withKI(0).withKD(1.2);
            //kG = 0.01, but angle not correct, so we have to manually implement it

            public static final double ROLLER_STATOR_CURRENT_LIMIT = 0;
            public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 0;

            public static final InvertedValue ROLLER_INVERTED = InvertedValue.Clockwise_Positive;
            public static final NeutralModeValue ROLLER_NEUTRAL_MODE = NeutralModeValue.Brake;

            public static final Slot0Configs ROLLER_PID = new Slot0Configs()
            .withKV(0).withKS(0)
            .withKP(0).withKI(0).withKD(0);
      }

      public static final class Misc {
        public static final int GYRO_PORT = 0;

      }
      public static final class Robot{
            public static final double SHOOTER_HEIGHT = 0.725; //m
      }

}
