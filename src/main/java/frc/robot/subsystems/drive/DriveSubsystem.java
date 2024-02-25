package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.ReplanningConfig;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;
import frc.robot.lib.math.SwerveMath;

public class DriveSubsystem extends SubsystemBase{

    private SwerveDrivePoseEstimator odometry;

    private Module[] modules;
    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private boolean fieldCentric;

    public static enum DriveState {
        DRIVER,
        AIM_TO_SPEAKER,
        AIM_TO_AMP
    }
    public DriveState state;

    public DriveSubsystem() {
        gyroIO = new GyroIOPigeon2();

        fieldCentric = true;

        modules = new Module[] {
            new Module(0, Constants.Drive.CANBUS),
            new Module(1, Constants.Drive.CANBUS),
            new Module(2, Constants.Drive.CANBUS),
            new Module(3, Constants.Drive.CANBUS)
        };

        odometry = new SwerveDrivePoseEstimator(
            Constants.Drive.KINEMATICS,
            getGyroAngle(),
            getSwerveModulePositions(),
            new Pose2d(0, 0, getGyroAngle()),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(5.0, 5.0, 0.9)
            );

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(6.0, 0.0, 0.0),
                        Constants.Drive.MAX_VELOCITY,
                        Constants.Drive.DIAGONAL,
                        new ReplanningConfig()
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );


        state = DriveState.DRIVER;
    }

    /*
     * xSpeed: speed of the robot (Forward) (m/s)
     * ySpeed: speed of the robot (Side) (m/s)
     * omegaSpeed: speed of the rotation (Counterclockwise positive) (rad/s)
     */
    public void set(double xSpeed, double ySpeed, double omegaSpeed){
        set(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
    }

    private PIDController angleController = new PIDController(10, 0, 0);
    private double lastJoystickAngle = 0;

    /*
     * chassisSpeeds: Object that contains values for the Chassis Speeds
     */
    public void set(ChassisSpeeds chassisSpeeds) {
        double x = chassisSpeeds.vxMetersPerSecond;
        double y = chassisSpeeds.vyMetersPerSecond;

        double omega = chassisSpeeds.omegaRadiansPerSecond;

        Logger.recordOutput("Desired ZVelocity", omega);

        if(fieldCentric) {
            if(gyroInputs.zVelocity >= 0.10 || omega != 0) lastJoystickAngle = getYaw().getRadians();
            else omega = Math.abs(lastJoystickAngle - getYaw().getRadians()) < Constants.TAU/10 &&
            Math.sqrt(x*x+y*y) > 0.1 ?
            angleController.calculate(getYaw().getRadians(), lastJoystickAngle) :
            0;
            
            double angleDiff = Math.atan2(y, x) - odometry.getEstimatedPosition().getRotation().getRadians(); //difference between input angle and gyro angle gives desired field relative angle

            double r = Math.sqrt(x*x + y*y); //magnitude of translation vector
            x = r * Math.cos(angleDiff);
            y = r * Math.sin(angleDiff);
        }
        
        // //Repeated equations
        double a = omega * Constants.Drive.WIDTH/2;
        double b = omega * Constants.Drive.LENGTH/2;

        //The addition of the movement and rotational vector
        Translation2d[] t = new Translation2d[] {
            new Translation2d(x+b, y+a),
            new Translation2d(x-b, y+a),
            new Translation2d(x-b, y-a),
            new Translation2d(x+b, y-a),
        };

        SwerveModuleState[] setStates = new SwerveModuleState[t.length];
        for(int i = 0; i < t.length; i++) {
            setStates[i] = new SwerveModuleState(t[i].getNorm(), t[i].getAngle());
        }

        setStates = SwerveMath.normalize(setStates);

        setStates(setStates);
    }

    public void setState(SwerveModuleState state, int modIndex) {
        modules[modIndex].set(state);
    }

    public void setDriveState(DriveState state){
        this.state = state;
    }

    public void resetPose(Pose2d pose) {
        resetOdometryManualAngle(pose, getGyroAngle());
    }

    public void resetOdometryManualAngle(Pose2d pose, Rotation2d angle) {
        odometry.resetPosition(angle, getSwerveModulePositions(), pose);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Drive.KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setStates(Constants.Drive.KINEMATICS.toSwerveModuleStates(speeds));
    }

    public void setStates(SwerveModuleState[] states) {
        for(int i = 0; i < modules.length; i++) {
            Logger.recordOutput("Velocity/Mod"+i+"Velocity", states[i].speedMetersPerSecond);
            Logger.recordOutput("Velocity/Mod"+i+"Angle", NRUnits.logConstrainRad(states[i].angle.getRadians()+Constants.TAU));
            modules[i].set(states[i]);
        }
    }



    public void setState(int modIndex, SwerveModuleState state) {
        Logger.recordOutput("Velocity/Mod"+modIndex+"Velocity", state.speedMetersPerSecond);
        Logger.recordOutput("Velocity/Mod"+modIndex+"Angle", NRUnits.logConstrainRad(state.angle.getRadians()+Constants.TAU));
        modules[modIndex].set(state);
    }

    public SwerveModule getModule(int modIndex) {
        return modules[modIndex].getModule();
    }


    public void setVoltageStates(double voltage){
        for(int i = 2; i < 4; i++){ //Setting only the back 2 motors
            modules[i].setBoltage(voltage);
        }
    }


    // True if the robot is field-centric, false for robot-centric
    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    // //Sets the angle of the robot in radians
    // public void setGyro(Rotation2d angle) {
    //     gyroIO.setYaw(angle.getDegrees());
    // }

    //Zeroes the yaw (Rotational direction)
    public void zeroYaw() {
        gyroIO.setYaw(0);
    }

    //Gets estimated Robot Pose (Includes vision integration (located in robot))
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    //Returns the Robot's yaw orientation in radians (Contstrained)
    // DO NOT USE
    public Rotation2d getGyroAngle() {
        return Rotation2d.fromRadians(NRUnits.constrainRad(getYaw().getRadians()));
    }

    public double getZVelocity(){
        return gyroInputs.zVelocity;
    }

    // DO NOT USE
    //Returns the gyro's yaw orientation in radians (Rotation Horizontal)
    public Rotation2d getYaw(){
        return Rotation2d.fromRadians(gyroInputs.yaw);
    }

    //Returns the gyro's pitch (Front/backflips)
    public Rotation2d getPitch(){
        return Rotation2d.fromRadians(gyroInputs.pitch);
    }

    //Returns the gyro's roll (Rolling over)
    public Rotation2d getRoll(){
        return Rotation2d.fromRadians(gyroInputs.roll);
    }

    public DriveState getDriveState(){
        return this.state;
    }

    public void updateOdometryWithVision(Pose2d visionPose, double timeStamp) {
        odometry.addVisionMeasurement(visionPose, timeStamp);
    }

    public void setVisionMeasurementStdDevs(Matrix<N3, N1> stds) {
        odometry.setVisionMeasurementStdDevs(stds);
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        ChassisSpeeds speeds = Constants.Drive.KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
        Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        translation = new Translation2d(translation.getNorm(), Rotation2d.fromRadians(translation.getAngle().getRadians() + getGyroAngle().getRadians()));
        speeds.vxMetersPerSecond = translation.getX();
        speeds.vyMetersPerSecond = translation.getY();
        return speeds;
    }

    public void setAngle(Rotation2d angle) {
        Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
        odometry.resetPosition(getGyroAngle(), getSwerveModulePositions(), pose);
    }

    public void zeroAngle() {
        setAngle(DriverStation.getAlliance().get() == Alliance.Blue ? Rotation2d.fromRadians(0) :
        Rotation2d.fromRadians(Constants.TAU/2));
    }

    @Override
    public void periodic(){
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for(Module module : modules) {
            module.periodic();
        }

        odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroAngle(), getSwerveModulePositions());
        Logger.recordOutput("FPGATimestamp", Timer.getFPGATimestamp());

        Pose2d pose = getPose();

        Logger.recordOutput("Pose", pose);
        Logger.recordOutput("GetGyroAngle", getGyroAngle().getRadians());
    }
}
