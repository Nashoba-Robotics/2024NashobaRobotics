package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsytem;

public class TestServoCommand extends Command {
    
    private ClimberSubsytem climb;

    public TestServoCommand(ClimberSubsytem climb) {
        this.climb = climb;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("ServoPos", 0);
    }

    @Override
    public void execute() {
        climb.setServo(SmartDashboard.getNumber("ServoPos", 0));
    }

    @Override
    public void end(boolean interrupted) {
        climb.setServo(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
