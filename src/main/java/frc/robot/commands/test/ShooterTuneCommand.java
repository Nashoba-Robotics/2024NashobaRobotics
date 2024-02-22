package frc.robot.commands.test;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterTuneCommand extends Command {
    
    private ArmSubsystem arm;

    private Slot0Configs config;

    public ShooterTuneCommand(ArmSubsystem arm) {
        this.arm = arm;

        config = new Slot0Configs();
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Rot/Sec", 0);

        SmartDashboard.putNumber("kV", 0);
        SmartDashboard.putNumber("kS", 0);
        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);
    }

    @Override
    public void execute() {
        arm.setShooterSpeed(Rotation2d.fromRotations(SmartDashboard.getNumber("Rot/Sec", 0)));

        if(SmartDashboard.getNumber("kV", 0) != config.kV) {
            config.kV = SmartDashboard.getNumber(("kV"), 0);
            arm.getShooterMotor().getConfigurator().apply(config);
        }
        if(SmartDashboard.getNumber("kS", 0) != config.kS) {
            config.kS = SmartDashboard.getNumber(("kS"), 0);
            arm.getShooterMotor().getConfigurator().apply(config);
        }
        if(SmartDashboard.getNumber("kP", 0) != config.kP) {
            config.kP = SmartDashboard.getNumber(("kP"), 0);
            arm.getShooterMotor().getConfigurator().apply(config);
        }
        if(SmartDashboard.getNumber("kI", 0) != config.kI) {
            config.kI = SmartDashboard.getNumber(("kI"), 0);
            arm.getShooterMotor().getConfigurator().apply(config);
        }
        if(SmartDashboard.getNumber("kD", 0) != config.kD) {
            config.kD = SmartDashboard.getNumber(("kD"), 0);
            arm.getShooterMotor().getConfigurator().apply(config);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShooterPercent(0);
    }

}
