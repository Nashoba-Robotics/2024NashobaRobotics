package frc.robot.commands;

import java.time.temporal.IsoFields;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ToggleFieldCentricCommand extends Command{
    DriveSubsystem drive = RobotContainer.drive;
    @Override
    public void initialize() {
        drive.setFieldCentric(!drive.isFieldCentric());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
