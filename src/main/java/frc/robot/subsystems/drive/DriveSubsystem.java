package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


    public DriveSubsystem() {
        gyroIO = new GyroIOPigeon2();

        fieldCentric = true;

        modules = new Module[] {
            new Module(0, Constants.Drive.CANBUS),
            new Module(1, Constants.Drive.CANBUS),
            new Module(2, Constants.Drive.CANBUS),
            new Module(3, Constants.Drive.CANBUS)
        };

        odometry = new SwerveDrivePoseEstimator(Constants.Drive.KINEMATICS, getGyroAngle(), getSwerveModulePositions(), new Pose2d(0, 0, getGyroAngle()));

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
    }


    public void set(ChassisSpeeds chassisSpeeds) {
        double x = chassisSpeeds.vxMetersPerSecond;
        double y = chassisSpeeds.vyMetersPerSecond;

        double omega = chassisSpeeds.omegaRadiansPerSecond;

        if(fieldCentric) {
            double angleDiff = Math.atan2(y, x) - getGyroAngle().getRadians(); //difference between input angle and gyro angle gives desired field relative angle
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

    public void set(SwerveModuleState[] states) {
        for(int i = 0; i < modules.length; i++) {
            modules[i].set(states[i]);
        }
    }

    public void set(SwerveModuleState state, int modIndex) {
        modules[modIndex].set(state);
    }

    public void resetPose(Pose2d pose) {
        resetOdometryManualAngle(pose, getGyroAngle());
    }

    private boolean resetting = false;
    public void resetOdometryManualAngle(Pose2d pose, Rotation2d angle) {
        resetting = true;
        odometry.resetPosition(angle, getSwerveModulePositions(), pose);
        resetting = false;
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

    public void setVoltageStates(double voltage){
        for(int i = 2; i < 4; i++){ //Setting only the back 2 motors
            modules[i].setBoltage(voltage);
        }
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    //radians
    public void setGyro(double angle) {
        gyroIO.setYaw(angle * 180 / Math.PI);
    }

    public void zeroYaw() {
        gyroIO.setYaw(0);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromRadians(NRUnits.constrainRad(getYaw().getRadians()));
    }

    public Rotation2d getYaw(){
        return Rotation2d.fromRadians(gyroInputs.yaw);
    }

    public Rotation2d getPitch(){
        return Rotation2d.fromRadians(gyroInputs.pitch);
    }

    public Rotation2d getRoll(){
        return Rotation2d.fromRadians(gyroInputs.roll);
    }

    public void updateOdometryWithVision(Pose2d visionPose, double timeStamp) {
        odometry.addVisionMeasurement(visionPose, timeStamp);
    }

    @Override
    public void periodic(){
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for(Module module : modules) {
            module.periodic();
        }

        if(!resetting) odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroAngle(), getSwerveModulePositions());
        Pose2d pose = getPose();

        Logger.recordOutput("Pose", pose);
        Logger.recordOutput("GetGyroAngle", getGyroAngle().getRadians());
    }
}
