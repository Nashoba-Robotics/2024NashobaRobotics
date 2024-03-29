package frc.robot.commands.setters.units.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsytem;

public class ServoToClimbPrep extends Command{
    ClimberSubsytem climber = RobotContainer.climber;

    @Override
    public void execute() {
        climber.setServo(0.8);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
