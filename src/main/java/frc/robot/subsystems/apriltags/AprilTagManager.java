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

        if(inputs.frontLeftHasTarget && inputs.frontLeftPos != null)
            Logger.recordOutput("RobotPos (Camera)", inputs.frontLeftPos.toPose2d());
    }

    //Returns whether or not we can see at least 1 april tag
    public static boolean hasLeftTarget(){
        return inputs.frontLeftHasTarget;
    }
    //Returns the timestamp of a frame (Only used for vision integration into odometry)
    public static double getLeftTimestamp(){
        return inputs.frontLeftTimeStamp;
    }
    //Returns the ambiguity ratio of the targets (High ambiguity = bad)
    public static double getLeftAmbiguity(){
        return inputs.leftAmbiguity;
    }
    //Returns the yaw of the "best" target (Defined by PhotonLib)
    public static double getLeftTargetYaw(){
        return inputs.leftYaw;
    }

    //Returns the robot position as a Pose3d
    public static Pose3d getLeftRobotPos(){
        return inputs.frontLeftPos;
    }

    //Returns whether or not we can see at least 1 april tag
    public static boolean hasRightTarget(){
        return inputs.rightHasTarget;
    }
    //Returns the timestamp of a frame (Only used for vision integration into odometry)
    public static double getRightTimestamp(){
        return inputs.rightTimeStamp;
    }
    //Returns the ambiguity ratio of the targets (High ambiguity = bad)
    public static double getRightAmbiguity(){
        return inputs.rightAmbiguity;
    }
    //Returns the yaw of the "best" target (Defined by PhotonLib)
    public static double getRightTargetYaw(){
        return inputs.rightYaw;
    }

    //Returns the robot position as a Pose3d
    public static Pose3d getRightRobotPos(){
        return inputs.rightPos;
    }
}
