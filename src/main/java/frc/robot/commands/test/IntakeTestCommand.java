package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeTestCommand extends Command{
    IntakeSubsystem intake;

    public IntakeTestCommand(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Intake Speed", 0);
    }

    @Override
    public void execute() {
        double speed = SmartDashboard.getNumber("Intake Speed", 0);
        intake.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
