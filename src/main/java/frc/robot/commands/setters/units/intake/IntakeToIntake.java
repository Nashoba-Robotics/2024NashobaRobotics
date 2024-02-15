package frc.robot.commands.setters.units.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeToIntake extends Command{
    IntakeSubsystem intake = RobotContainer.intake;

    public IntakeToIntake(){
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setSpeed(Presets.Intake.INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
