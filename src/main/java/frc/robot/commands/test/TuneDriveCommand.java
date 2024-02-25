package frc.robot.commands.test;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TuneDriveCommand extends Command {

    private DriveSubsystem drive;

    private SwerveModule mod;
    private int modIndex;

    private Slot0Configs moveConfig;
    private Slot0Configs turnConfig;

    public TuneDriveCommand(DriveSubsystem drive) {
        this.drive = drive;
        modIndex = 0;
        mod = this.drive.getModule(modIndex);

        moveConfig = new Slot0Configs();
        turnConfig = new Slot0Configs();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("modIndex", 0);
        
        SmartDashboard.putNumber("moveP", 0);
        SmartDashboard.putNumber("moveI", 0);
        SmartDashboard.putNumber("moveD", 0);
        SmartDashboard.putNumber("moveS", 0);
        SmartDashboard.putNumber("moveV", 0);

        SmartDashboard.putNumber("turnP", 0);
        SmartDashboard.putNumber("turnI", 0);
        SmartDashboard.putNumber("turnD", 0);
        SmartDashboard.putNumber("turnS", 0);
        SmartDashboard.putNumber("turnV", 0);

        SmartDashboard.putNumber("angle", 0);
        SmartDashboard.putNumber("speed", 0);

        mod.getDriveMotor().getConfigurator().apply(moveConfig);
        mod.getSteerMotor().getConfigurator().apply(turnConfig);
    }

    @Override
    public void execute() {
        SwerveModuleState state = new SwerveModuleState(
            SmartDashboard.getNumber("speed", 0),
            Rotation2d.fromDegrees(
                SmartDashboard.getNumber("angle", 0)
            )
            );

        Logger.recordOutput("SetVelocity", state.speedMetersPerSecond);
        Logger.recordOutput("SetAngle", state.angle.getRadians());

        mod.apply(state, DriveRequestType.Velocity, SteerRequestType.MotionMagicExpo);

        int currModIndex = (int)SmartDashboard.getNumber("modIndex", 0);

        double currMoveP = SmartDashboard.getNumber("moveP", 0);
        double currMoveI = SmartDashboard.getNumber("moveI", 0);
        double currMoveD = SmartDashboard.getNumber("moveD", 0);
        double currMoveS = SmartDashboard.getNumber("moveS", 0);
        double currMoveV = SmartDashboard.getNumber("moveV", 0);

        double currTurnP = SmartDashboard.getNumber("turnP", 0);
        double currTurnI = SmartDashboard.getNumber("turnI", 0);
        double currTurnD = SmartDashboard.getNumber("turnD", 0);
        double currTurnS = SmartDashboard.getNumber("turnS", 0);
        double currTurnV = SmartDashboard.getNumber("turnV", 0);

        if(currModIndex != modIndex) {
            modIndex = currModIndex;
            mod = drive.getModule(modIndex);
        }

        if(currMoveP != moveConfig.kP) {
            moveConfig = moveConfig.withKP(currMoveP);
            mod.getDriveMotor().getConfigurator().apply(moveConfig);
        }
        if(currMoveI != moveConfig.kI) {
            moveConfig = moveConfig.withKI(currMoveI);
            mod.getDriveMotor().getConfigurator().apply(moveConfig);
        }
        if(currMoveD != moveConfig.kD) {
            moveConfig = moveConfig.withKD(currMoveD);
            mod.getDriveMotor().getConfigurator().apply(moveConfig);
        }
        if(currMoveS != moveConfig.kS) {
            moveConfig = moveConfig.withKS(currMoveS);
            mod.getDriveMotor().getConfigurator().apply(moveConfig);
        }
        if(currMoveV != moveConfig.kV) {
            moveConfig = moveConfig.withKV(currMoveV);
            mod.getDriveMotor().getConfigurator().apply(moveConfig);
        }

        if(currTurnP != turnConfig.kP) {
            turnConfig = turnConfig.withKP(currTurnP);
            mod.getSteerMotor().getConfigurator().apply(turnConfig);
        }
        if(currTurnI != turnConfig.kI) {
            turnConfig = turnConfig.withKI(currTurnI);
            mod.getSteerMotor().getConfigurator().apply(turnConfig);
        }
        if(currTurnD != turnConfig.kD) {
            turnConfig = turnConfig.withKD(currTurnD);
            mod.getSteerMotor().getConfigurator().apply(turnConfig);
        }
        if(currTurnS != turnConfig.kS) {
            turnConfig = turnConfig.withKS(currTurnS);
            mod.getSteerMotor().getConfigurator().apply(turnConfig);
        }
        if(currTurnV != turnConfig.kV) {
            turnConfig = turnConfig.withKV(currTurnV);
            mod.getSteerMotor().getConfigurator().apply(turnConfig);
        }

    }

    @Override
    public void end(boolean interrupted) {
        mod.apply(
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            DriveRequestType.Velocity,
            SteerRequestType.MotionMagicExpo);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
