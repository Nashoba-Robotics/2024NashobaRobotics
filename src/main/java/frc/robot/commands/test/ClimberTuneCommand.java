package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsytem;

public class ClimberTuneCommand extends Command{
    ClimberSubsytem climber;

    ClimberTuneCommand(ClimberSubsytem climber){
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Climber Pos", 0);

    }

    @Override
    public void execute() {
        double pos = SmartDashboard.getNumber("Climber Pos", 0);
        climber.setClimberPos(pos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
