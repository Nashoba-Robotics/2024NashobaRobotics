package frc.robot.commands.test;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class LoaderTuneCommand extends Command{
    ArmSubsystem arm;
    double lastkG = 0.01;
    double lastkS = 0.02;
    double lastKV = 0.845;
    double lastKP = 45;
    double lastKD = 1.2;
    public LoaderTuneCommand(ArmSubsystem arm){
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.setLoaderPivotRotor(Rotation2d.fromRadians(0));

        SmartDashboard.putNumber("Loader Deg", 0);
        SmartDashboard.putNumber("Loader Pivot Speed", 0);
        SmartDashboard.putNumber("Pivot kG", lastkG);
        SmartDashboard.putNumber("Pivot kS", lastkS);
        SmartDashboard.putNumber("Pivot kV", lastKV);
        SmartDashboard.putNumber("Pivot kP", lastKP);
        SmartDashboard.putNumber("Pivot kD", lastKD);

        arm.setLoaderPivotkG(lastkG);
        arm.setLoaderPivotkS(lastkS);
        arm.setLoaderPivotkV(lastKV);
        arm.setLoaderPivotkP(lastKP);
        arm.setLoaderPivotkD(lastKD);
    }

    @Override
    public void execute() {
        Rotation2d targetPos = Rotation2d.fromDegrees(SmartDashboard.getNumber("Loader Deg", 0));
        arm.setLoaderPivot(targetPos);

        // double speed = SmartDashboard.getNumber("Loader Pivot Speed", 0);
        // arm.setLoaderPivotSpeed(speed);
        Logger.recordOutput("Desired Angle", targetPos.getRadians());

        double kG = SmartDashboard.getNumber("Pivot kG", 0);
        if(kG != lastkG){
            arm.setLoaderPivotkG(kG);
            lastkG = kG;
        }
        double kS = SmartDashboard.getNumber("Pivot kS", 0);
        if(kS != lastkS){
            arm.setLoaderPivotkS(kS);
            lastkS = kS;
        }
        double kV = SmartDashboard.getNumber("Pivot kV", 0);
        if(kV != lastKV){
            arm.setLoaderPivotkV(kV);
            lastKV = kV;
        } 
        double kP = SmartDashboard.getNumber("Pivot kP", 0);
        if(kP != lastKP){
            arm.setLoaderPivotkP(kP);
            lastKP = kP;
        }
        double kD = SmartDashboard.getNumber("Pivot kD", 0);
        if(kD != lastKD){
            arm.setLoaderPivotkD(kD);
            lastKD = kD;
        }

        Logger.recordOutput("Error", Math.abs(targetPos.getRadians()-arm.getLoaderPivotAngle().getRadians()));
    }

    @Override
    public void end(boolean interrupted) {
        arm.setLoaderPivotSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
