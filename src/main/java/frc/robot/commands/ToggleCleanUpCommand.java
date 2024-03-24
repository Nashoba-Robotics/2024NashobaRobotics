package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Governor;

public class ToggleCleanUpCommand extends Command{
    @Override
    public void initialize() {
        Governor.cleanUp = true;
    }

    @Override
    public void end(boolean interrupted) {
        Governor.cleanUp = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
