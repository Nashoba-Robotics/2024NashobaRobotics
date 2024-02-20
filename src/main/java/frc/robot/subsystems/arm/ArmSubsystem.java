package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    
    private ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();


    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Arm/Arm", armInputs);
    }

    public ArmSubsystem() {
        armIO = new ArmIOTalonFX();
    }

    //Sets the pivot position. (0 should be horizontal to the ground)
    public void setArmPivot(Rotation2d position) {
        armIO.setAngle(position);
    }


    public void setArmPivotRotor(Rotation2d rotorPos){
        armIO.setPivotRotorPos(rotorPos);
    }


    //Returns the arm pivot angle as a Rotation2D

    public Rotation2d getArmPivotAngle() {
        return Rotation2d.fromRadians(armInputs.pivotRotorPosition);
    }

    //Sets the speed of the shooter. Units don't matter b/c Rotation2D
    public void setShooterSpeed(Rotation2d speed) {
        Logger.recordOutput("SetShooterSpeed", speed.getRadians());
        armIO.setShooterSpeed(speed);
    }

    public void setShooterPercent(double speed){
        armIO.setShooterPercent(speed);
    }

    //Returns the speed of the shooter
    public Rotation2d getShooterSpeed() {
        return Rotation2d.fromRadians(armInputs.topShooterSpeed);
    }
    
    public void setArmPivotSpeed(double speed){
        armIO.setPivotSpeed(speed);
    }
    public void setArmPivotkG(double kG){
        armIO.setPivotkG(kG);
    }
    public void setArmPivotkS(double kS){
        armIO.setPivotkS(kS);
    }
    public void setArmPivotkV(double kV){
        armIO.setPivotkV(kV);
    }
    public void setArmPivotkP(double kP){
        armIO.setPivotkP(kP);
    }
    public void setArmPivotkD(double kD){
        armIO.setPivotkD(kD);
    }

    public TalonFX getShooterMotor() {
        return armIO.getShooterMotor();
    }
}
