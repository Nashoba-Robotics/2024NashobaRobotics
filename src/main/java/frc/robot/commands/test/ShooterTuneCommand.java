package frc.robot.commands.test;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterTuneCommand extends Command{
    ArmSubsystem arm;

    double lastkS, lastkV, lastkP, lastkD;
    public ShooterTuneCommand(ArmSubsystem arm){
        this.arm = arm;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Shooter Speed", 0);

        SmartDashboard.putNumber("Shooter kS", 0);  //0.025
        SmartDashboard.putNumber("Shooter kV", 0);
        SmartDashboard.putNumber("Shooter kP", 0);
        SmartDashboard.putNumber("SHooter kD", 0);
    }

    @Override
    public void execute() {
        // double kS = SmartDashboard.getNumber("Shooter kS", 0);
        // if(kS != lastkS){
        //     arm.setShooterkS(kS);
        //     lastkS = kS;
        // }
        // double kV = SmartDashboard.getNumber("Shooter kV", 0);
        // if(kV != lastkV){
        //     arm.setShooterkV(kV);
        //     lastkV = kV;
        // }
        // double kP = SmartDashboard.getNumber("Shooter kP", 0);
        // if(kP != lastkP){
        //     arm.setShooterkP(kP);
        //     lastkP = kP;
        // }
        // double kD = SmartDashboard.getNumber("Shooter kD", 0);
        // if(kD != lastkD){
        //     arm.setShooterkD(kD);
        //     lastkD = kD;
        // }

        double speed = SmartDashboard.getNumber("Shooter Speed", 0);
        // arm.setShooterSpeed(Rotation2d.fromRotations(speed));
        arm.setShooterPercentOutput(speed);

        Logger.recordOutput("Current Speed", arm.getShooterSpeed().getRotations());
        Logger.recordOutput("Target Speed", speed);

        double loaderSpeed = SmartDashboard.getNumber("Loader", 0);
        // arm.setLoaderSpeed(Rotation2d.fromRotations(loaderSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShooterSpeed(Rotation2d.fromRotations(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
