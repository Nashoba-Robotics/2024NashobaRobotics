package frc.robot.commands.auto.amp;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ToAmpCommand extends SequentialCommandGroup {
    
    public ToAmpCommand() {
        // Load the path we want to pathfind to and follow
        PathPlannerPath path = PathPlannerPath.fromPathFile("Amp");

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand0 = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        addCommands(
            pathfindingCommand0
        );
    }

}
