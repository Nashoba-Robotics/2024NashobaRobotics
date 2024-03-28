package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Presets;

public class ArmSubsystem extends SubsystemBase{
    
    private ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private double lastTime;
    private boolean rampDown = false;
    private boolean rampUp = false;
    private boolean ramped = false;


    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Arm", armInputs);

        if(ramped){
            lastTime = System.currentTimeMillis();
            ramped = false;
        }
    }

    public ArmSubsystem() {
        armIO = new ArmIOTalonFX();
        lastTime = System.currentTimeMillis();
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
        rampUp = false;
        rampDown = false;
        Logger.recordOutput("SetShooterSpeed", speed.getRadians());
        armIO.setShooterSpeed(speed);
    }

    public void setShooterPercent(double speed){
        armIO.setShooterPercent(speed);
    }

    public void setIdleSpeed(double speed){
        if(!rampDown) ramped = true;
        rampDown = true;
        rampUp = false;
    }

    public void rampToSpeed(){
        if(!rampUp){
            ramped = true;
            lastTime = System.currentTimeMillis();
        } 
        rampUp = true;
        rampDown = false;
        
        //Ramp from idle speed = 
        if (rampUp) double time = System.currentTimeMillis() - lastTime;
        double speed = 500 * (0.2 + 0.75/3 * time);
        if(speed >= Presets.Arm.SPEAKER_SPEED.getRadians()){
            speed = Presets.Arm.SPEAKER_SPEED.getRadians();
            rampUp = false;
        }

        armIO.setShooterSpeed(Rotation2d.fromRadians(speed));
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
    public void setShooterPercentOutput(double speed){
        armIO.setShooterPercentOutput(speed);
    }
}
