package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class JangsCommand extends Command{
    ArmSubsystem arm;
    IntakeSubsystem intake;
    public JangsCommand(ArmSubsystem arm, IntakeSubsystem intake){
        this.arm = arm;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Shooter", 0);
        SmartDashboard.putNumber("Loader", 0);
        SmartDashboard.putNumber("Intake", 0);
    }
    @Override
    public void execute() {
        double shooter = SmartDashboard.getNumber("Shooter", 0);
        arm.setShooterPercentOutput(shooter);
        double loader = SmartDashboard.getNumber("Loader", 0);
        arm.setLoaderPercentOutput(loader);
        double intake = SmartDashboard.getNumber("Intake", 0);
        this.intake.setSpeed(intake);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShooterPercentOutput(0);
        arm.setLoaderPercentOutput(0);
        intake.setSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
