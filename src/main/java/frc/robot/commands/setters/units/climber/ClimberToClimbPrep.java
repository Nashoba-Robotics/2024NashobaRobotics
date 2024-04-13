package frc.robot.commands.setters.units.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsytem;

public class ClimberToClimbPrep extends Command {
    
    private ClimberSubsytem climber = RobotContainer.climber;

    public ClimberToClimbPrep() {
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setPos(Presets.Climber.CLIMB_PREP_POS);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climber.getPos().getRadians() - Presets.Climber.CLIMB_PREP_POS.getRadians()) < Presets.Climber.POS_TOLERANCE.getRadians()
        || Math.abs(RobotContainer.joysticks.getOperatorController().getY()) > 0.1;
    }

}
