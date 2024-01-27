package frc.robot.subsystems.apriltags;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagManager extends SubsystemBase {
    private AprilTagIO io;
    private static AprilTagIOInputsAutoLogged inputs = new AprilTagIOInputsAutoLogged();

    public AprilTagManager(){
        io = new AprilTagIOPhotonVision();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Camera", inputs);

        if(inputs.hasTarget && inputs.pos != null)
            Logger.recordOutput("RobotPos (Camera)", inputs.pos.toPose2d());
    }

    //Returns whether or not we can see at least 1 april tag
    public static boolean hasTarget(){
        return inputs.hasTarget;
    }
    //Returns the timestamp of a frame (Only used for vision integration into odometry)
    public static double getTimestamp(){
        return inputs.timeStamp;
    }
    //Returns the ambiguity ratio of the targets (High ambiguity = bad)
    public static double getAmbiguity(){
        return inputs.ambiguity;
    }
    //Returns the yaw of the "best" target (Defined by PhotonLib)
    public static double getTargetYaw(){
        return inputs.yaw;
    }

    //Returns the robot position as a Pose3d
    public static Pose3d getRobotPos(){
        return inputs.pos;
    }
}
