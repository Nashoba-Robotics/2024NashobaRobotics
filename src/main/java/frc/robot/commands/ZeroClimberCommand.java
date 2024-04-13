package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsytem;

public class ZeroClimberCommand extends Command{
    ClimberSubsytem climber;
    public ZeroClimberCommand(ClimberSubsytem climber){
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void execute() {
        climber.setSpeed(-0.05);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setRotor(new Rotation2d());
        climber.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return climber.getStatorCurrent() <= -0.4 || climber.getStatorCurrent() >= 6;
    }
}
