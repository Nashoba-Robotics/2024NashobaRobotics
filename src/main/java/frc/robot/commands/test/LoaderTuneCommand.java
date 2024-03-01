package frc.robot.commands.test;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class LoaderTuneCommand extends Command{
    LoaderSubsystem loader;
    double lastkG = 0.01;
    double lastkS = 0.02;
    double lastKV = 0.845;
    double lastKP = 45;
    double lastKD = 1.2;
    public LoaderTuneCommand(LoaderSubsystem loader){
        this.loader = loader;
        addRequirements(loader);
    }

    @Override
    public void initialize() {
        loader.setPivotRotor(Rotation2d.fromRadians(0));

        SmartDashboard.putNumber("Loader Deg", 0);
        SmartDashboard.putNumber("Loader Roll Speed", 0);
        SmartDashboard.putNumber("Loader Pivot Speed", 0);
        SmartDashboard.putNumber("Pivot kG", lastkG);
        SmartDashboard.putNumber("Pivot kS", lastkS);
        SmartDashboard.putNumber("Pivot kV", lastKV);
        SmartDashboard.putNumber("Pivot kP", lastKP);
        SmartDashboard.putNumber("Pivot kD", lastKD);

        loader.setPivotkG(lastkG);
        loader.setPivotkS(lastkS);
        loader.setPivotkV(lastKV);
        loader.setPivotkP(lastKP);
        loader.setPivotkD(lastKD);
    }

    @Override
    public void execute() {
        Rotation2d targetPos = Rotation2d.fromDegrees(SmartDashboard.getNumber("Loader Deg", 0));
        loader.setPivot(targetPos);

        double loaderSpeed = SmartDashboard.getNumber("Loader Roll Speed", 0);
        loader.setRollerSpeed(loaderSpeed);

        // double speed = SmartDashboard.getNumber("Loader Pivot Speed", 0);
        // arm.setLoaderPivotSpeed(speed);
        Logger.recordOutput("Desired Angle", targetPos.getRadians());

        double kG = SmartDashboard.getNumber("Pivot kG", 0);
        if(kG != lastkG){
            loader.setPivotkG(kG);
            lastkG = kG;
        }
        double kS = SmartDashboard.getNumber("Pivot kS", 0);
        if(kS != lastkS){
            loader.setPivotkS(kS);
            lastkS = kS;
        }
        double kV = SmartDashboard.getNumber("Pivot kV", 0);
        if(kV != lastKV){
            loader.setPivotkV(kV);
            lastKV = kV;
        } 
        double kP = SmartDashboard.getNumber("Pivot kP", 0);
        if(kP != lastKP){
            loader.setPivotkP(kP);
            lastKP = kP;
        }
        double kD = SmartDashboard.getNumber("Pivot kD", 0);
        if(kD != lastKD){
            loader.setPivotkD(kD);
            lastKD = kD;
        }

        Logger.recordOutput("Error", Math.abs(targetPos.getRadians()-loader.getPivotAngle().getRadians()));
    }

    @Override
    public void end(boolean interrupted) {
        loader.setPivotSpeed(0);
        loader.setRollerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
