package frc.robot.commands.test;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTestCommand extends SequentialCommandGroup {

    public AutoTestCommand() {
        addCommands(
            new PathPlannerAuto("")
        );
    }

}