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
    private double startSpeed;
    private boolean rampDown = false;
    private boolean rampUp = false;
    final double rampUpTime = 3000;//ms
    final double rampDownTime = 3000;//ms



    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Arm", armInputs);
    }

    public ArmSubsystem() {
        armIO = new ArmIOTalonFX();
        lastTime = System.currentTimeMillis();
    }

    //Sets the pivot position. (0 should be horizontal to the ground)
    public void setArmPivot(Rotation2d position) {
        Logger.recordOutput("ArmSetAngle", position);
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
        rampDown = false;
        rampUp = false;
        armIO.setShooterPercent(speed);
    }

    public void setIdleSpeed(double idleSpeed){
        if(!rampDown){
            rampDown = true;
            rampUp = false;

            startSpeed = getShooterSpeed().getRadians()/500;
            lastTime = System.currentTimeMillis();
        } 

        double time = System.currentTimeMillis() - lastTime;

        double targetSpeed = idleSpeed * 500;
        double calculatedSpeed = 500 * (startSpeed - (startSpeed-idleSpeed)/rampDownTime * time);

        if (calculatedSpeed > targetSpeed){
            targetSpeed = calculatedSpeed;
            armIO.setShooterSpeed(Rotation2d.fromRadians(targetSpeed));
        } 
        else if(calculatedSpeed <= targetSpeed){
            armIO.setShooterPercent(idleSpeed);
        }

        
    }

    public void rampToSpeed(){
        if(!rampUp){
            rampUp = true;
            rampDown = false;
            startSpeed = getShooterSpeed().getRadians()/500;
            lastTime = System.currentTimeMillis();
        } 
        
        double time = (System.currentTimeMillis() - lastTime);
        double speed = Presets.Arm.SPEAKER_SPEED.getRadians();
        double targetSpeed = 500 * (startSpeed + Presets.Arm.SPEAKER_SPEED.getRadians()/500/rampUpTime * time);
        
        if (targetSpeed < speed){
            speed = targetSpeed;
        } 
        if(targetSpeed >= Presets.Arm.SPEAKER_SPEED.getRadians()){
            speed = Presets.Arm.SPEAKER_SPEED.getRadians();
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

    public TalonFX getPivotMotor() {
        return armIO.getPivotMotor();
    }
}
