package frc.robot.commands.setters.units.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsytem;

public class ClimberToTrap extends Command {
    
    private ClimberSubsytem climber = RobotContainer.climber;

    public ClimberToTrap() {
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setClimberPos(Presets.Climber.TRAP_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climber.getLeftRad() - Presets.Climber.TRAP_POS.getRadians()) < Presets.Climber.POS_TOLERANCE;
    }

}
