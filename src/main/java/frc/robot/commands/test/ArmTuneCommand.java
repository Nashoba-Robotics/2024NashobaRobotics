package frc.robot.commands.test;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Tabs;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmTuneCommand extends Command{
    ArmSubsystem arm;
    double lastkG, lastkV, lastkS, lastkP, lastkD;
    public ArmTuneCommand(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // SmartDashboard.putNumber("kG", lastkG);
        // SmartDashboard.putNumber("kS", lastkS);
        // SmartDashboard.putNumber("kV", lastkV);
        // SmartDashboard.putNumber("kP", lastkP);
        // SmartDashboard.putNumber("kD", lastkD);

        SmartDashboard.putNumber("Arm Deg", 0);
        SmartDashboard.putNumber("Shoot Speed", 0);

        // Tabs.addTab("Arm");

        // Tabs.putNumber("Arm", "kG", lastkG);
        // Tabs.putNumber("Arm", "kS", lastkS);
        // Tabs.putNumber("Arm", "kV", lastkV);
        // Tabs.putNumber("Arm", "kP", lastkP);
        // Tabs.putNumber("Arm", "kD", lastkD);

        // Tabs.putNumber("Arm", "Arm Deg", 0);
        // Tabs.putNumber("Arm", "Shoot Speed", 0);

    }

    @Override
    public void execute() {
        // // double kG = SmartDashboard.getNumber("kG", 0);
        // double kG = Tabs.getNumber("Arm", "kG");
        // if(kG != lastkG){
        //     arm.setArmPivotkG(kG);
        //     lastkG = kG;
        // }
        // // double kS = SmartDashboard.getNumber("kS", 0);
        // double kS = Tabs.getNumber("Arm", "kS");
        // if(kS != lastkS){
        //     arm.setArmPivotkS(kS);
        //     lastkS = kS;
        // }
        // // double kV = SmartDashboard.getNumber("kV", 0);
        // double kV = Tabs.getNumber("Arm", "kV");
        // if(kV != lastkV){
        //     arm.setArmPivotkV(kV);
        //     lastkV = kV;
        // }
        // // double kP = SmartDashboard.getNumber("kP", 0);
        // double kP = Tabs.getNumber("Arm", "kP");
        // if(kP != lastkP){
        //     arm.setArmPivotkP(kP);
        //     lastkP = kP;
        // }
        // // double kD = SmartDashboard.getNumber("kD", 0);
        // double kD = Tabs.getNumber("Arm", "kD");
        // if(kD != lastkD){
        //     arm.setArmPivotkD(kD);
        //     lastkD = kD;
        // }

        double angle = SmartDashboard.getNumber("Arm Deg", 0);
        // double angle = Tabs.getNumber("Arm", "Arm Deg");
        arm.setArmPivot(Rotation2d.fromDegrees(angle));

        // double shootSpeed = Tabs.getNumber("Arm", "Shoot Speed");
        double shootSpeed = SmartDashboard.getNumber("Shoot Speed", 0);
        arm.setShooterPercent(shootSpeed);

        // Logger.recordOutput("Andyson", arm.getArmPivotAngle().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        // Tabs.putNumber("Arm", "kG", lastkG);
        // Tabs.putNumber("Arm", "kS", lastkS);
        // Tabs.putNumber("Arm", "kV", lastkV);
        // Tabs.putNumber("Arm", "kP", lastkP);
        // Tabs.putNumber("Arm", "kD", lastkD);

        Tabs.putNumber("Arm", "Arm Deg", 0);
        Tabs.putNumber("Arm", "Shoot Speed", 0);

        arm.setArmPivotSpeed(0);
        arm.setShooterPercent(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
