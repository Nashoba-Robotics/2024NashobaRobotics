package frc.robot.commands.setters.units.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class IntakeToShoot extends Command{
    IntakeSubsystem intake = RobotContainer.intake;
    LoaderSubsystem loader = RobotContainer.loader;

    public IntakeToShoot(){
        addRequirements(intake);
    }

    @Override
    public void execute() {
        double intakeSpeed = Presets.Intake.INTAKE_SPEED;
        intake.setSpeed(intakeSpeed);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
