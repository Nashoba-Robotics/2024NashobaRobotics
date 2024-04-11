package frc.robot.commands.setters.units.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsytem;

public class ClimberToClimb extends Command {
    
    private ClimberSubsytem climber = RobotContainer.climber;

    public ClimberToClimb() {
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setClimberPos(Presets.Climber.CLIMB_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climber.getClimberPos().getRadians() - Presets.Climber.CLIMB_POS.getRadians()) < Presets.Climber.POS_TOLERANCE.getRadians();
    }

}
