package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    
    private ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private LoaderIO loaderIO;
    private LoaderIOInputsAutoLogged loaderInputs = new LoaderIOInputsAutoLogged();

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        loaderIO.updateInputs(loaderInputs);
        Logger.processInputs("Arm/Arm", armInputs);
        Logger.processInputs("Arm/Loader", loaderInputs);
    }

    public ArmSubsystem() {
        armIO = new ArmIOTalonFX();
        loaderIO = new LoaderIOTalonFX();
    }

    public void setArmPivot(Rotation2d position) {
        armIO.setAngle(position);
    }

    public Rotation2d getArmPivotAngle() {
        return Rotation2d.fromRadians(armInputs.pivotRotorPosition);
    }

    public void setShooterSpeed(Rotation2d speed) {
        armIO.setShooterSpeed(speed);
    }

    public Rotation2d getShooterSpeed() {
        return Rotation2d.fromRadians(armInputs.shooterSpeed);
    }

    public void setLoaderPivot(Rotation2d position) {
        loaderIO.setPivotPosition(position);
    }

    public Rotation2d getLoaderPivotAngle() {
        return Rotation2d.fromRadians(loaderInputs.pivotPosition);
    }

    public void setLoaderSpeed(Rotation2d speed) {
        loaderIO.setRollerSpeed(speed);
    }

    public Rotation2d getLoaderSpeed() {
        return Rotation2d.fromRadians(loaderInputs.rollerVelocity);
    }

}
