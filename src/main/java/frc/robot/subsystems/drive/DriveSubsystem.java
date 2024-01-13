package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;
import frc.robot.lib.math.SwerveMath;

public class DriveSubsystem extends SubsystemBase{

    private SwerveDriveOdometry odometry;

    private Module[] modules;

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private boolean fieldCentric;

    public DriveSubsystem() {
        gyroIO = new GyroIOPigeon2();

        fieldCentric = false;

        modules = new Module[] {
            new Module(0, Constants.Drive.CANBUS),
            new Module(1, Constants.Drive.CANBUS),
            new Module(2, Constants.Drive.CANBUS),
            new Module(3, Constants.Drive.CANBUS)
        };

        odometry = new SwerveDriveOdometry(Constants.Drive.KINEMATICS, getGyroAngle(), getSwerveModulePositions());
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
        
        //Repeated equations
        double a = omega * Constants.Drive.WIDTH/2;
        double b = omega * Constants.Drive.LENGTH/2;

        //The addition of the movement and rotational vector
        Translation2d[] t = new Translation2d[] {
            new Translation2d(x-b, y-a),
            new Translation2d(x+b, y-a),
            new Translation2d(x+b, y+a),
            new Translation2d(x-b, y+a),
        };

        SwerveModuleState[] setStates = new SwerveModuleState[t.length];
        for(int i = 0; i < t.length; i++) {
            setStates[i] = new SwerveModuleState(t[i].getNorm(), t[i].getAngle());
        }

        setStates = SwerveMath.normalize(setStates);

        set(setStates);
    }

    public void set(SwerveModuleState[] states) {
        for(int i = 0; i < modules.length; i++) {
            modules[i].set(states[i]);
        }
    }

    public void set(SwerveModuleState state, int modIndex) {
        modules[modIndex].set(state);
    }

    private boolean resetting = false;
    public void resetOdometry(Pose2d pose, Rotation2d angle) {
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

    public void setStates(SwerveModuleState[] states) {
        for(int i = 0; i < modules.length; i++) {
            // SmartDashboard.putNumber("SetAngle"+i, states[i].angle.getDegrees());
            modules[i].set(states[i]);
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
        return odometry.getPoseMeters();
    }

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromRadians(NRUnits.constrainRad(getYaw().getRadians()) * Constants.TAU / 360);
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

    @Override
    public void periodic(){
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for(Module module : modules) {
            module.periodic();
        }

        if(!resetting) odometry.update(getGyroAngle(), getSwerveModulePositions());
        Pose2d pose = odometry.getPoseMeters();

        Logger.recordOutput("Drive/Odometry/X", pose.getX());
        Logger.recordOutput("Drive/Odometry/Y", pose.getY());
        Logger.recordOutput("Drive/Odometry/Angle", pose.getRotation().getDegrees());
    }
}
