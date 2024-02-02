package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO{
    //2 for shooter rollers (Krakens)
    //1 for pivot (Kraken maybe)
    TalonFX shooter, shooter2;
    TalonFX pivot;

    public ArmIOTalonFX(){
        shooter = new TalonFX(Constants.Arm.SHOOTER_PORT, "RIO");
        shooter2 = new TalonFX(Constants.Arm.SHOOTER_PORT_2, "RIO");

        pivot = new TalonFX(Constants.Arm.PIVOT_PORT, "RIO");
    }

    public void updateInputs(ArmIOInputs inputs){

    }

    @Override
    public void setAngle(double angle){

    }

    @Override
    public void setShooterSpeed(double speed){
        
    }
}
