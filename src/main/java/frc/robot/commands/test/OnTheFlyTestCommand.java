package frc.robot.commands.test;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnTheFlyTestCommand extends SequentialCommandGroup {

    public OnTheFlyTestCommand() {

        Pose2d targetPose = new Pose2d(5, 5, Rotation2d.fromDegrees(180));

        PathConstraints constraints0 = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints0,
                0.0,
                0.0
        );

        addCommands(
            pathfindingCommand
        );
    }

}