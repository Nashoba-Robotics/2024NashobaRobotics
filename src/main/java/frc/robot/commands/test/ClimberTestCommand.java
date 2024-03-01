package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsytem;

public class ClimberTestCommand extends Command{
    ClimberSubsytem climber;

    public ClimberTestCommand(ClimberSubsytem climber){
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        CommandJoystick controller = RobotContainer.joysticks.getOperatorController();

        double speed = -controller.getY() * 0.3;
        climber.setClimberSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setClimberSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
